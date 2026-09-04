#include "globals.h"
#include "board_definition.h"

#ifdef SD_LOGGING
#include <SPI.h>
#ifdef __SD_H__
  #include <SD.h>
#else
  #include "SdFat.h"
#endif
#include "SD_logger.h"
#include "logger.h"
#include "rtc_common.h"
#include "maths.h"
#include <elapsedMillis.h>

//List of logger field names. This must be in the same order and length as logger_updateLogdataCSV()
constexpr char header_0[] = "secl";
constexpr char header_1[] = "status1";
constexpr char header_2[] = "engine";
constexpr char header_3[] = "Sync Loss #";
constexpr char header_4[] = "MAP";
constexpr char header_5[] = "IAT(C)";
constexpr char header_6[] = "CLT(C)";
constexpr char header_7[] = "Battery Correction";
constexpr char header_8[] = "Battery V";
constexpr char header_9[] = "AFR";
constexpr char header_10[] = "EGO Correction";
constexpr char header_11[] = "IAT Correction";
constexpr char header_12[] = "WUE Correction";
constexpr char header_13[] = "RPM";
constexpr char header_14[] = "Accel. Correction";
constexpr char header_15[] = "Gamma Correction";
constexpr char header_16[] = "VE1";
constexpr char header_17[] = "VE2";
constexpr char header_18[] = "AFR Target";
constexpr char header_19[] = "TPSdot";
constexpr char header_20[] = "Advance Current";
constexpr char header_21[] = "TPS";
constexpr char header_22[] = "Loops/S";
constexpr char header_23[] = "Free RAM";
constexpr char header_24[] = "Boost Target";
constexpr char header_25[] = "Boost Duty";
constexpr char header_26[] = "status2";
constexpr char header_27[] = "rpmDOT";
constexpr char header_28[] = "Eth%";
constexpr char header_29[] = "Flex Fuel Correction";
constexpr char header_30[] = "Flex Adv Correction";
constexpr char header_31[] = "IAC Steps/Duty";
constexpr char header_32[] = "testoutputs";
constexpr char header_33[] = "AFR2";
constexpr char header_34[] = "Baro";
constexpr char header_35[] = "AUX_IN 0";
constexpr char header_36[] = "AUX_IN 1";
constexpr char header_37[] = "AUX_IN 2";
constexpr char header_38[] = "AUX_IN 3";
constexpr char header_39[] = "AUX_IN 4";
constexpr char header_40[] = "AUX_IN 5";
constexpr char header_41[] = "AUX_IN 6";
constexpr char header_42[] = "AUX_IN 7";
constexpr char header_43[] = "AUX_IN 8";
constexpr char header_44[] = "AUX_IN 9";
constexpr char header_45[] = "AUX_IN 10";
constexpr char header_46[] = "AUX_IN 11";
constexpr char header_47[] = "AUX_IN 12";
constexpr char header_48[] = "AUX_IN 13";
constexpr char header_49[] = "AUX_IN 14";
constexpr char header_50[] = "AUX_IN 15";
constexpr char header_51[] = "TPS ADC";
constexpr char header_52[] = "Errors";
constexpr char header_53[] = "PW";
constexpr char header_54[] = "PW2";
constexpr char header_55[] = "PW3";
constexpr char header_56[] = "PW4";
constexpr char header_57[] = "status3";
constexpr char header_58[] = "Engine Protect";
constexpr char header_59[] = "";
constexpr char header_60[] = "Fuel Load";
constexpr char header_61[] = "Ign Load";
constexpr char header_62[] = "Dwell Requested";
constexpr char header_63[] = "Idle Target (RPM)";
constexpr char header_64[] = "MAP DOT";
constexpr char header_65[] = "VVT1 Angle";
constexpr char header_66[] = "VVT1 Target";
constexpr char header_67[] = "VVT1 Duty";
constexpr char header_68[] = "Flex Boost Adj";
constexpr char header_69[] = "Baro Correction";
constexpr char header_70[] = "VE Current";
constexpr char header_71[] = "ASE Correction";
constexpr char header_72[] = "Vehicle Speed";
constexpr char header_73[] = "Gear";
constexpr char header_74[] = "Fuel Pressure";
constexpr char header_75[] = "Oil Pressure";
constexpr char header_76[] = "WMI PW";
constexpr char header_77[] = "status4";
constexpr char header_78[] = "VVT2 Angle";
constexpr char header_79[] = "VVT2 Target";
constexpr char header_80[] = "VVT2 Duty";
constexpr char header_81[] = "outputs";
constexpr char header_82[] = "Fuel Temp";
constexpr char header_83[] = "Fuel Temp Correction";
constexpr char header_84[] = "Advance 1";
constexpr char header_85[] = "Advance 2";
constexpr char header_86[] = "SD Status";
constexpr char header_87[] = "EMAP";
constexpr char header_88[] = "Fan Duty";
constexpr char header_89[] = "AirConStatus";
constexpr char header_90[] = "Dwell Actual";
constexpr char header_91[] = "status5";
constexpr char header_92[] = "Knock Count";
constexpr char header_93[] = "Knock Retard";
constexpr char header_94[] = "PW5";
constexpr char header_95[] = "PW6";
constexpr char header_96[] = "PW7";
constexpr char header_97[] = "PW8";
constexpr char header_98[] = "System Temp";
/*
constexpr char header_99[] = "";
constexpr char header_100[] = "";
constexpr char header_101[] = "";
constexpr char header_102[] = "";
constexpr char header_103[] = "";
constexpr char header_104[] = "";
constexpr char header_105[] = "";
constexpr char header_106[] = "";
constexpr char header_107[] = "";
constexpr char header_108[] = "";
constexpr char header_109[] = "";
constexpr char header_110[] = "";
constexpr char header_111[] = "";
constexpr char header_112[] = "";
constexpr char header_113[] = "";
constexpr char header_114[] = "";
constexpr char header_115[] = "";
constexpr char header_116[] = "";
constexpr char header_117[] = "";
constexpr char header_118[] = "";
constexpr char header_119[] = "";
constexpr char header_120[] = "";
constexpr char header_121[] = "";
*/

constexpr const char* header_table[] = {  header_0,\
                                              header_1,\
                                              header_2,\
                                              header_3,\
                                              header_4,\
                                              header_5,\
                                              header_6,\
                                              header_7,\
                                              header_8,\
                                              header_9,\
                                              header_10,\
                                              header_11,\
                                              header_12,\
                                              header_13,\
                                              header_14,\
                                              header_15,\
                                              header_16,\
                                              header_17,\
                                              header_18,\
                                              header_19,\
                                              header_20,\
                                              header_21,\
                                              header_22,\
                                              header_23,\
                                              header_24,\
                                              header_25,\
                                              header_26,\
                                              header_27,\
                                              header_28,\
                                              header_29,\
                                              header_30,\
                                              header_31,\
                                              header_32,\
                                              header_33,\
                                              header_34,\
                                              header_35,\
                                              header_36,\
                                              header_37,\
                                              header_38,\
                                              header_39,\
                                              header_40,\
                                              header_41,\
                                              header_42,\
                                              header_43,\
                                              header_44,\
                                              header_45,\
                                              header_46,\
                                              header_47,\
                                              header_48,\
                                              header_49,\
                                              header_50,\
                                              header_51,\
                                              header_52,\
                                              header_53,\
                                              header_54,\
                                              header_55,\
                                              header_56,\
                                              header_57,\
                                              header_58,\
                                              header_59,\
                                              header_60,\
                                              header_61,\
                                              header_62,\
                                              header_63,\
                                              header_64,\
                                              header_65,\
                                              header_66,\
                                              header_67,\
                                              header_68,\
                                              header_69,\
                                              header_70,\
                                              header_71,\
                                              header_72,\
                                              header_73,\
                                              header_74,\
                                              header_75,\
                                              header_76,\
                                              header_77,\
                                              header_78,\
                                              header_79,\
                                              header_80,\
                                              header_81,\
                                              header_82,\
                                              header_83,\
                                              header_84,\
                                              header_85,\
                                              header_86,\
                                              header_87,\
                                              header_88,\
                                              header_89,\
                                              header_90,\
                                              header_91,\
                                              header_92,\
                                              header_93,\
                                              header_94,\
                                              header_95,\
                                              header_96,\
                                              header_97,\
                                              header_98,\
                                              /*
                                              header_99,\
                                              header_100,\
                                              header_101,\
                                              header_102,\
                                              header_103,\
                                              header_104,\
                                              header_105,\
                                              header_106,\
                                              header_107,\
                                              header_108,\
                                              header_109,\
                                              header_110,\
                                              header_111,\
                                              header_112,\
                                              header_113,\
                                              header_114,\
                                              header_115,\
                                              header_116,\
                                              header_117,\
                                              header_118,\
                                              header_119,\
                                              header_120,\
                                              header_121,\
                                              */
                                            };

static_assert(sizeof(header_table) == (sizeof(char*) * SD_LOG_NUM_FIELDS), "Number of header table titles must match number of log fields");

SdExFat sd;
ExFile logFile;
RingBuf<ExFile, RING_BUF_CAPACITY> rb;
uint8_t SD_status = SD_STATUS_OFF;
uint16_t currentLogFileNumber;
bool manualLogActive = false;
uint32_t logStartTime = 0; //In ms
elapsedMillis msSinceLastSDSync;

void initSD()
{
  //Set default state to ready. If any stage of the init fails, this will be changed
  SD_status = SD_STATUS_READY; 

  //Set the RTC callback. This is used to set the correct timestamp on file creation and sync operations
  FsDateTime::setCallback(dateTime);

  // Initialise the SD.
  if (!sd.begin(SD_CONFIG)) 
  {
    //sd.initErrorHalt(&Serial);
    //if (sdErrorCode() == SD_CARD_ERROR_CMD0) { SD_status = SD_STATUS_ERROR_NO_CARD;
    SD_status = SD_STATUS_ERROR_NO_CARD;
  }
  
  //Set the TunerStudio status variable
  setTS_SD_status();
}

bool createLogFile()
{
  //TunerStudio only supports 8.3 filename format. 
  char filenameBuffer[13]; //8 + 1 + 3 + 1
  bool returnValue = false;

  //Saving this in case we ever go back to the datestamped filename
  /*
  //Filename format is: YYYY-MM-DD_HH.MM.SS.csv
  char intBuffer[5];
  itoa(rtc_getYear(), intBuffer, 10);
  strcpy(filenameBuffer, intBuffer);
  strcat(filenameBuffer, "-");
  itoa(rtc_getMonth(), intBuffer, 10);
  strcat(filenameBuffer, intBuffer);
  strcat(filenameBuffer, "-");
  itoa(rtc_getDay(), intBuffer, 10);
  strcat(filenameBuffer, intBuffer);
  strcat(filenameBuffer, "_");
  itoa(rtc_getHour(), intBuffer, 10);
  strcat(filenameBuffer, intBuffer);
  strcat(filenameBuffer, ".");
  itoa(rtc_getMinute(), intBuffer, 10);
  strcat(filenameBuffer, intBuffer);
  strcat(filenameBuffer, ".");
  itoa(rtc_getSecond(), intBuffer, 10);
  strcat(filenameBuffer, intBuffer);
  strcat(filenameBuffer, ".csv");
  */

  //Lookup the next available file number
  currentLogFileNumber = getNextSDLogFileNumber();

  //Create the filename
  //sprintf(filenameBuffer, "%s%04d.%s", LOG_FILE_PREFIX, currentLogFileNumber, LOG_FILE_EXTENSION);
  if(currentLogFileNumber > MAX_LOG_FILES) { currentLogFileNumber = 1; } //If we've run out of file numbers, start again from 1
  snprintf(filenameBuffer, 13, "%s%04d.%s", LOG_FILE_PREFIX, currentLogFileNumber, LOG_FILE_EXTENSION);

  logFile.close();
  if (logFile.open(filenameBuffer, O_RDWR | O_CREAT | O_TRUNC)) 
  {
    returnValue = true;
  }

  return returnValue;
}

uint16_t getNextSDLogFileNumber()
{
  uint16_t nextFileNumber = 1;
  char filenameBuffer[13]; //8 + 1 + 3 + 1
  sprintf(filenameBuffer, "%s%04d.%s", LOG_FILE_PREFIX, nextFileNumber, LOG_FILE_EXTENSION);

  //Lookup the next available file number
  while( (nextFileNumber < MAX_LOG_FILES) && (sd.exists(filenameBuffer)) )
  {
    nextFileNumber++;
    sprintf(filenameBuffer, "%s%04d.%s", LOG_FILE_PREFIX, nextFileNumber, LOG_FILE_EXTENSION);
  }

  return nextFileNumber;
}

bool getSDLogFileDetails(uint8_t* buffer, uint16_t logNumber)
{
  bool fileFound = false;

  if(logFile.isOpen()) { endSDLogging(); }

  char filenameBuffer[13]; //8 + 1 + 3 + 1
  if(logNumber > MAX_LOG_FILES) { logNumber = MAX_LOG_FILES; } //If we've run out of file numbers, start again from 1
  snprintf(filenameBuffer, 13, "%s%04d.%s", LOG_FILE_PREFIX, logNumber, LOG_FILE_EXTENSION);
  
  if(sd.exists(filenameBuffer))
  {
    fileFound = true;

    logFile = sd.open(filenameBuffer, O_RDONLY);
    //Copy the filename into the buffer. Note we do not copy the termination character or the fullstop
    for(byte i=0; i<12; i++)
    {
      //We don't copy the fullstop to the buffer
      //As TS requires 8.3 filenames, it's always in the same spot
      if(i < 8) { buffer[i] = filenameBuffer[i]; } //Everything before the fullstop
      else if(i > 8) { buffer[i-1] = filenameBuffer[i]; } //Everything after the fullstop
    }

    //Maintenance check, truncate the file. This will usually do nothing, but in the case where a prior log was interrupted, this will truncate the file
    //Due to overhead, only bother doing this if the engine isn't running
    if(currentStatus.RPM == 0) { logFile.truncate(); }

    //Is File or ignore
    buffer[11] = 1;

    //No idea
    buffer[12] = 0;

    //5 bytes for FAT creation date/time
    uint16_t pDate = 0;
    uint16_t pTime = 0;
    logFile.getCreateDateTime(&pDate, &pTime);
    buffer[13] = 0; //Not sure what this byte is for yet
    buffer[14] = lowByte(pTime);
    buffer[15] = highByte(pTime);
    buffer[16] = lowByte(pDate);
    buffer[17] = highByte(pDate);

    //Sector number (4 bytes) - This byte order might be backwards
    uint32_t sector = logFile.firstSector();
    buffer[18] = ((sector) & UINT8_MAX);
    buffer[19] = ((sector >> 8) & UINT8_MAX);
    buffer[20] = ((sector >> 16) & UINT8_MAX);
    buffer[21] = ((sector >> 24) & UINT8_MAX);

    //Unsure on the below 6 bytes, possibly last accessed or modified date/time?
    buffer[22] = 0;
    buffer[23] = 0;
    buffer[24] = 0;
    buffer[25] = 0;
    buffer[26] = 0;
    buffer[27] = 0;

    //File size (4 bytes). Little endian
    uint32_t size = logFile.fileSize();
    buffer[28] = ((size) & UINT8_MAX);
    buffer[29] = ((size >> 8) & UINT8_MAX);
    buffer[30] = ((size >> 16) & UINT8_MAX);
    buffer[31] = ((size >> 24) & UINT8_MAX);

  }

  return fileFound;
}

void readSDSectors(uint8_t* buffer, uint32_t sectorNumber, uint16_t sectorCount)
{
  sd.card()->readSectors(sectorNumber, buffer, sectorCount);
}

// Forward declare
void writeSDLogHeader();

void beginSDLogging()
{
  if(SD_status == SD_STATUS_READY)
  {
    SD_status = SD_STATUS_ACTIVE; //Set the status as being active so that entries will begin to be written. This will be updated below if there is an error

    // Open or create file - truncate existing file.
    if (!createLogFile()) 
    {
      SD_status = SD_STATUS_ERROR_NO_WRITE;
      setTS_SD_status();
      return;
    }

    //Perform pre-allocation on card. This dramatically improves write speed
    if (!logFile.preAllocate(SD_LOG_FILE_SIZE)) 
    {
      SD_status = SD_STATUS_ERROR_NO_SPACE;
      setTS_SD_status();
      return;
    }

    //initialise the RingBuf.
    rb.begin(&logFile);

    //Write a header row
    writeSDLogHeader();

    //Note the start time
    logStartTime = millis();
  }
}

void endSDLogging()
{
  if(SD_status == SD_STATUS_ACTIVE)
  {
    // Write any RingBuf data to file.
    rb.sync();
    logFile.truncate();
    logFile.rewind();
    logFile.close();
    logFile.sync(); //This is required to update the sd object. Without this any subsequent logfiles will overwrite this one

    SD_status = SD_STATUS_READY;
    setTS_SD_status();
  }
}

// Forward declare
void checkForSDStart();
void checkForSDStop();

void writeSDLogEntry()
{
  //Check if we're already running a log
  if(SD_status == SD_STATUS_READY)
  {
    //Log not currently running, check if it should be
    checkForSDStart();
  }

  if(SD_status == SD_STATUS_ACTIVE)
  {
    //Check that there is enough free space in the ring buffer to write the entry
    if(rb.bytesFree() > SD_LOG_ENTRY_TOTAL_BYTES)
    {
      //Write the timestamp (x.yyy seconds format)
      uint32_t duration = millis() - logStartTime;
      uint32_t seconds = duration / 1000;
      uint32_t milliseconds = duration % 1000;
      rb.print(seconds);
      rb.print('.');
      if (milliseconds < 100) { rb.print("0"); }
      if (milliseconds < 10) { rb.print("0"); }
      rb.print(milliseconds);
      rb.print(',');

      //Write the line to the ring buffer
      for(byte x=0; x<SD_LOG_NUM_FIELDS; x++)
      {
        #if FPU_MAX_SIZE >= 32
          float entryValue = getReadableFloatLogEntry(x);
          if(IS_INTEGER(entryValue)) 
          { 
            uint16_t entryValueInt = (uint16_t)entryValue;
            if(entryValueInt <= UCHAR_MAX) { rb.print((uint8_t)entryValueInt); }
            else { rb.print(entryValueInt); }
          }
          else { rb.print(entryValue); }
        #else
          rb.print(getReadableLogEntry(x));
        #endif
        if(x < (SD_LOG_NUM_FIELDS - 1)) { rb.print(","); }
      }
      rb.println("");
    }

    //Check if write to SD from ringbuffer is needed
    //We write to SD when there is more than 1 sector worth of data in the ringbuffer and there is not already a write being performed
    if( (rb.bytesUsed() >= SD_SECTOR_SIZE) && !logFile.isBusy())
    {
      uint16_t bytesWritten = rb.writeOut(SD_SECTOR_SIZE); 
      
      //Make sure that the entire sector was written successfully
      if (SD_SECTOR_SIZE != bytesWritten) 
      {
        SD_status = SD_STATUS_ERROR_WRITE_FAIL;
      }
    }

    //Check whether we should stop logging
    checkForSDStop();

    //Check whether the file is full (IE When there is not enough room to write 1 more sector)
    if( (logFile.dataLength() - logFile.curPosition()) < SD_SECTOR_SIZE)
    {
      //Provided the conditions for logging are still met, a new file will be created the next time writeSDLogEntry is called
      endSDLogging();
      beginSDLogging();
    }
  }
  setTS_SD_status();
}

void writeSDLogHeader()
{
  //Write header for Time field
  rb.print("Time,");

  //WRite remaining fields based on log definitions
  for(byte x=0; x<SD_LOG_NUM_FIELDS; x++)
  {
    rb.print(header_table[x]);
    if(x < (SD_LOG_NUM_FIELDS - 1)) { rb.print(","); }
  }
  rb.println("");
}

//Sets the status variable for TunerStudio
void setTS_SD_status()
{
  currentStatus.sdCardPresent = (SD_status != SD_STATUS_ERROR_NO_CARD);
  currentStatus.sdCardType = 1U; // CARD is SDHC
  currentStatus.sdCardReady = true;
  currentStatus.sdCardLogging = (SD_status == SD_STATUS_ACTIVE);
  currentStatus.sdCardError = (SD_status >= SD_STATUS_ERROR_NO_FS);
  currentStatus.sdCardFS = 1U; // CARD has a FAT32 filesystem (Though this will be exFAT)
  currentStatus.sdCardUnused = false; //Unused bit is always 0
}

/** 
 * Checks whether the SD logging should be started based on the logging trigger conditions
 */
void checkForSDStart()
{
  //Logging can only start if we're in the ready state
  //We must check the SD_status each time to prevent trying to init a new log file multiple times

  if(configPage13.onboard_log_file_style > 0)
  {
    //Check for enable at boot
    if( (configPage13.onboard_log_trigger_boot) && (SD_status == SD_STATUS_READY) )
    {
      //Check that we're not already finished the logging
      if((millis() / 1000) <= configPage13.onboard_log_tr1_duration)
      {
        beginSDLogging(); //Setup the log file, preallocation, header row
      }    
    }

    //Check for RPM based Enable
    if( (configPage13.onboard_log_trigger_RPM) && (SD_status == SD_STATUS_READY) )
    {
      if( (currentStatus.RPMdiv100 >= configPage13.onboard_log_tr2_thr_on) && (currentStatus.RPMdiv100 <= configPage13.onboard_log_tr2_thr_off) ) //Need to check both on and off conditions to prevent logging starting and stopping continually
      {
        beginSDLogging(); //Setup the log file, preallocation, header row
      }
    }

    //Check for engine protection based enable
    if((configPage13.onboard_log_trigger_prot) && (SD_status == SD_STATUS_READY) )
    {
      if(currentStatus.engineProtect.isActive())
      {
        beginSDLogging(); //Setup the log file, preallocation, header row
      }
    }

    if( (configPage13.onboard_log_trigger_Vbat) && (SD_status == SD_STATUS_READY) )
    {

    }

    if((configPage13.onboard_log_trigger_Epin) && (SD_status == SD_STATUS_READY) )
    {
      if(digitalRead(pinSDEnable) == LOW)
      {
        beginSDLogging(); //Setup the log file, preallocation, header row
      }
    }
  }
}

/** 
 * Checks whether the SD logging should be stopped, based on the logging trigger conditions
 */
void checkForSDStop()
{
  //Check the various conditions to see if we should stop logging
  bool log_boot = false;
  bool log_RPM = false;
  bool log_prot = false;
  bool log_Vbat = false;
  bool log_Epin = false;

  //Logging only needs to be stopped if already active
  if(SD_status == SD_STATUS_ACTIVE)
  {
    //Check for enable at boot
    if(configPage13.onboard_log_trigger_boot)
    {
      //Check if we're past the logging duration
      if((millis() / 1000) <= configPage13.onboard_log_tr1_duration)
      {
        log_boot = true;
      }
    }
    if(configPage13.onboard_log_trigger_RPM)
    {
      if( (currentStatus.RPMdiv100 >= configPage13.onboard_log_tr2_thr_on) && (currentStatus.RPMdiv100 <= configPage13.onboard_log_tr2_thr_off) )
      {
        log_RPM = true;
      }
    }
    if(configPage13.onboard_log_trigger_prot)
    {
      if(currentStatus.engineProtect.isActive())
      {
        log_prot = true;
      }
    }
    if(configPage13.onboard_log_trigger_Vbat)
    {

    }

    //External Pin
    if(configPage13.onboard_log_trigger_Epin)
    {
      if(digitalRead(pinSDEnable) == LOW)
      {
        log_Epin = true;
      }
    }

    //Check all conditions to see if we should stop logging
    if( (log_boot == false) && (log_RPM == false) && (log_prot == false) && (log_Vbat == false) && (log_Epin == false) && (manualLogActive == false) )
    {
      endSDLogging();
    }
    //ALso check whether logging has been disabled entirely
    if(configPage13.onboard_log_file_style == 0) { endSDLogging(); }
  }

  
}

bool syncSDLog()
{     
  if( (SD_status == SD_STATUS_ACTIVE) && (!logFile.isBusy()) && (!sd.isBusy()) )
  {
    logFile.sync();
    return true;
  }
  return false;
}

/** 
 * Will perform a complete format of the SD card to ExFAT. 
 * This will delete all files and create a new empty file system.
 * The SD status will be set to busy when this happens to prevent any other operations
 */
void formatExFat()
{
  bool result = false;

  //Set the SD status to busy
  currentStatus.sdCardReady = false;

  logFile.close();

  if (sd.cardBegin(SD_CONFIG)) 
  {
    if(sd.format()) 
    {
      if (sd.volumeBegin())
      {
        result = true;
      }
    }
  }

  if(result == false) { SD_status = SD_STATUS_ERROR_FORMAT_FAIL; }
  else { currentStatus.sdCardReady = true; }
}

/**
 * @brief Deletes a log file from the SD card
 * 
 * Log files all have the same name with a 4 digit number at the end (Eg SPD_0001.csv). TS sends the 4 digits as ASCII characters and they are combined here with the logfile prefix
 * 
 * @param log1 
 * @param log2 
 * @param log3 
 * @param log4 
 */
void deleteLogFile(char log1, char log2, char log3, char log4)
{
  char logFileName[13];
  strcpy(logFileName, LOG_FILE_PREFIX);
  logFileName[4] = log1;
  logFileName[5] = log2;
  logFileName[6] = log3;
  logFileName[7] = log4;
  logFileName[8] = '.';
  strcpy(logFileName + 9, LOG_FILE_EXTENSION);
  //logFileName[8] = '\0';

  if(sd.exists(logFileName))
  {
    sd.remove(logFileName);
  }
}

// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  
  // Return date using FS_DATE macro to format fields.
  //*date = FS_DATE(year(), month(), day());
  *date = FS_DATE(rtc_getYear(), rtc_getMonth(), rtc_getDay());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(rtc_getHour(), rtc_getMinute(), rtc_getSecond());
  
  // Return low time bits in units of 10 ms.
  *ms10 = rtc_getSecond() & 1 ? 100 : 0;
}

uint32_t sectorCount()
{
  return sd.card()->sectorCount();
}

#endif
