#!/bin/bash
# uncomment to echo *fully* expanded script commands to terminal
# set -x


get_abs_filename() {
  # $1 : relative filename
  echo "$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
}

script_folder=$(get_abs_filename "$(dirname $(readlink -f $0))")

# Initialize variables with defaults
source_folder="$script_folder/../speeduino" # -s, --source
out_folder="$script_folder/.results"        # -o, --out
cppcheck_path=""                            # -c, --cppcheck
quiet=0                                     # -q, --quiet
output_xml=0                                # -x, --xml
update_baseline=0                           # -b, --update-baseline

function parse_command_line() {
   while [ $# -gt 0 ] ; do
    case "$1" in
      -s | --source) source_folder="$2" ;;
      -o | --out) out_folder="$2" ;;
      -c | --cppcheck) cppcheck_path="$2" ;;
      -q | --quiet) quiet=1 ;;
      -x | --xml) output_xml=1 ;;
      -b | --update-baseline) update_baseline=1 ;;
      -*) 
        echo "Unknown option: " $1
        exit 1
        ;;
    esac
    shift
  done
}

parse_command_line "$@"

# Have to use absolute paths for source:
# 1. CPPCheck (or the shell) expands globs to absolute paths.
# 2. CPPCheck matches paths from the command line using simple string comparisons
#   2.1. E.g. exclusion folders
source_folder=$(get_abs_filename "$source_folder")

# With no -c the binary comes from PATH; "${cppcheck_path}/cppcheck" would
# resolve to "/cppcheck", which is nobody's install and a confusing 127.
if [ -z "$cppcheck_path" ] ; then
  cppcheck_bin="cppcheck"
else
  cppcheck_bin="${cppcheck_path}/cppcheck"
fi

if ! command -v "$cppcheck_bin" > /dev/null 2>&1 ; then
  echo "cppcheck not found: $cppcheck_bin" >&2
  echo "Install it, or point at an install with -c /path/to/cppcheck-folder" >&2
  exit 127
fi

num_cores=`getconf _NPROCESSORS_ONLN`
let num_cores--

# --cppcheck-build-dir below is a cache, and a warm one silently changes the
# answer: an unchanged file is not re-analysed, so no .dump is produced for it,
# so the MISRA addon never sees it. A second run over identical sources reported
# 128 findings where the first reported 151. Start clean, and the number means
# something.
rm -rf "$out_folder"
mkdir -p "$out_folder"

# misra.json points at the rule texts with a path relative to the *current*
# directory, so the addon reports "rule-texts-file not found" against every
# finding unless the script happens to be run from misra/. Generate a copy with
# an absolute path instead - and a Windows-shaped one where cygpath exists,
# since cppcheck and its Python addon are native Windows binaries that cannot
# open a /c/... MSYS path.
rule_texts="$script_folder/misra_2012_text.txt"
if command -v cygpath > /dev/null 2>&1 ; then
  rule_texts=$(cygpath -m "$rule_texts")
fi
addon_config="$out_folder/misra.json"
echo "{\"script\": \"misra.py\",\"args\": [\"--rule-texts=$rule_texts\"]}" > "$addon_config"

cppcheck_parameters=( --inline-suppr
                      --language=c++
                      --enable=warning
                      --enable=information
                      --enable=performance
                      --enable=portability
                      --enable=style
                      --addon="$addon_config"
                      --suppressions-list="$script_folder/suppressions.txt"
                      --suppress=unusedFunction:*
                      --suppress=missingInclude:*
                      --suppress=missingIncludeSystem:*
                      --suppress=unmatchedSuppression:*
                      --suppress=cstyleCast:*
                      --platform=unix32
                      --cppcheck-build-dir="$out_folder"
                      -j "$num_cores"
                      -DCORE_STM32=1
                      -DSTM32F4
                      -DSTM32F407xx
                      -DARDUINO_ARCH_STM32
                      -DARDUINO=10808
                      # board_stm32_official.h #errors out without the first three,
                      # atomic.h without the fourth.
                      -DPLATFORMIO
                      -DUSBCON
                      -DUSBD_USE_CDC
                      -D__arm__
                      # This is defined in the Arduino core headers, which aren't included.
                      # cppcheck will not do type checking on unknown types.
                      # It's used a lot and it's unsigned, which can trigger a lot
                      # of type mismatch violations.
                      -Dbyte=uint8_t
                      -i $source_folder/src/SPIAsEEPROM
                      "$source_folder"
                      "$source_folder/*.ino")

cppcheck_out_file="$out_folder/results.txt"
if [ $output_xml -eq 1 ]; then
  cppcheck_out_file="$out_folder/results.xml"
  cppcheck_parameters+=(--xml)
fi

"$cppcheck_bin" ${cppcheck_parameters[@]} 2> $cppcheck_out_file
cppcheck_status=$?

# A crashed or aborted run leaves a partial results file, and the count below
# would then report a reassuring zero. Say so instead.
if [ $cppcheck_status -ne 0 ]; then
  echo "cppcheck exited $cppcheck_status - results are incomplete" >&2
  if [ $quiet -eq 0 ]; then
    cat "$cppcheck_out_file"
  fi
  exit $cppcheck_status
fi

# Count lines for Mandatory or Required rules.
# NOTE: this depends on the rule categories coming from misra_2012_text.txt, and
# the copy in this repo is a placeholder that says "No text specified" for nearly
# every rule (the real text is copyrighted). Until a real one is supplied this
# count is structurally zero - use the misra-c2012-* tags in results.txt instead.
error_count=`grep -i "Mandatory - \|Required - " < "$cppcheck_out_file" | wc -l`

if [ $quiet -eq 0 ]; then
  cat "$cppcheck_out_file"
fi
echo $error_count MISRA violations
echo $error_count > "$out_folder/error_count.txt"

# Compare the per-rule counts against the recorded baseline. Counts rather than
# file positions, so moving code around does not churn the file and only a
# violation actually being added shows up.
baseline_file="$script_folder/baseline.txt"
current_counts=$(grep -o "misra-c2012-[0-9.]*" "$cppcheck_out_file" | sort | uniq -c | awk '{print $2, $1}' | sort)

if [ $update_baseline -eq 1 ] ; then
  {
    grep '^#' "$baseline_file" 2> /dev/null
    echo "$current_counts"
  } > "$baseline_file.new"
  mv "$baseline_file.new" "$baseline_file"
  echo "Baseline updated: $(echo "$current_counts" | awk '{s+=$2} END {print s}') findings"
  exit 0
fi

if [ ! -f "$baseline_file" ] ; then
  echo "No baseline at $baseline_file - run with --update-baseline to create one" >&2
  exit 0
fi

regressions=0
while read -r rule count ; do
  [ -z "$rule" ] && continue
  was=$(grep -E "^$rule " "$baseline_file" | awk '{print $2}')
  was=${was:-0}
  if [ "$count" -gt "$was" ] ; then
    echo "REGRESSION: $rule went from $was to $count" >&2
    regressions=$((regressions + 1))
  fi
done <<< "$current_counts"

total_now=$(echo "$current_counts" | awk '{s+=$2} END {print s+0}')
total_was=$(grep -v '^#' "$baseline_file" | awk '{s+=$2} END {print s+0}')
echo "MISRA findings: $total_now (baseline $total_was)"

if [ $regressions -ne 0 ] ; then
  echo "$regressions rule(s) got worse. Fix them, or update the baseline deliberately." >&2
  exit 1
fi


exit 0