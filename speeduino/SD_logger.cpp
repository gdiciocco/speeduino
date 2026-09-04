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
constexpr const char* header_table[] = {
  "secl",
  "status1",
  "engine",
  "Sync Loss #",
  "MAP",
  "IAT(C)",
  "CLT(C)",
  "Battery Correction",
  "Battery V",
  "AFR",
  "EGO Correction",
  "IAT Correction",
  "WUE Correction",
  "RPM",
  "Accel. Correction",
  "Gamma Correction",
  "VE1",
  "VE2",
  "AFR Target",
  "TPSdot",
  "Advance Current",
  "TPS",
  "Loops/S",
  "Free RAM",
  "Boost Target",
  "Boost Duty",
  "status2",
  "rpmDOT",
  "Eth%",
  "Flex Fuel Correction",
  "Flex Adv Correction",
  "IAC Steps/Duty",
  "testoutputs",
  "AFR2",
  "Baro",
  "AUX_IN 0",
  "AUX_IN 1",
  "AUX_IN 2",
  "AUX_IN 3",
  "AUX_IN 4",
  "AUX_IN 5",
  "AUX_IN 6",
  "AUX_IN 7",
  "AUX_IN 8",
  "AUX_IN 9",
  "AUX_IN 10",
  "AUX_IN 11",
  "AUX_IN 12",
  "AUX_IN 13",
  "AUX_IN 14",
  "AUX_IN 15",
  "TPS ADC",
  "Errors",
  "PW",
  "PW2",
  "PW3",
  "PW4",
  "status3",
  "Engine Protect",
  "",
  "Fuel Load",
  "Ign Load",
  "Dwell Requested",
  "Idle Target (RPM)",
  "MAP DOT",
  "VVT1 Angle",
  "VVT1 Target",
  "VVT1 Duty",
  "Flex Boost Adj",
  "Baro Correction",
  "VE Current",
  "ASE Correction",
  "Vehicle Speed",
  "Gear",
  "Fuel Pressure",
  "Oil Pressure",
  "WMI PW",
  "status4",
  "VVT2 Angle",
  "VVT2 Target",
  "VVT2 Duty",
  "outputs",
  "Fuel Temp",
  "Fuel Temp Correction",
  "Advance 1",
  "Advance 2",
  "SD Status",
  "EMAP",
  "Fan Duty",
  "AirConStatus",
  "Dwell Actual",
  "status5",
  "Knock Count",
  "Knock Retard",
  "PW5",
  "PW6",
  "PW7",
  "PW8",
  "System Temp",
  // Spare slots: add the field name here and bump SD_LOG_NUM_FIELDS.
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
