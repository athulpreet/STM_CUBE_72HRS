/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
char buffer[64];
uint8_t seconds, minutes, hours, day, date, month, year;

// GPS DMA buffer and processing variables
#define GPS_BUFFER_SIZE 1024
uint8_t gps_dma_buffer[GPS_BUFFER_SIZE];
uint16_t gps_proc_index = 0;
uint8_t gps_line_buffer[256];
uint16_t gps_line_index = 0;

// GPS data
float gps_latitude = 0.0;
float gps_longitude = 0.0;
int gps_satellites = 0;
int gps_fix_quality = 0;
char gps_timestamp[10] = "000000.00";
bool gps_valid_data = false;
float gps_speed_kmh = 0.0;
char gps_date[7] = "000000";  // DDMMYY
bool gps_antenna_status = false;

// MCP7940N I2C address (7-bit address)
#define MCP7940N_ADDRESS 0x6F

// MCP7940N registers
#define MCP7940N_REG_SECONDS 0x00
#define MCP7940N_REG_MINUTES 0x01
#define MCP7940N_REG_HOURS   0x02
#define MCP7940N_REG_WKDAY   0x03
#define MCP7940N_REG_DATE    0x04
#define MCP7940N_REG_MONTH   0x05
#define MCP7940N_REG_YEAR    0x06
#define MCP7940N_REG_CONTROL 0x07
#define MCP7940N_REG_OSCTRIM 0x08






// --------------------- Circular Queue Implementation ---------------------
// Queue parameters for 72 hours of logging
#define QUEUE_CAPACITY 10  // (72 hours * 60 minutes * 60 seconds) / 5 seconds 51840
#define ENTRY_SIZE 64         // Fixed size of each log entry in bytes
#define INDEX_FILENAME "queue.idx"
#define DATA_FILENAME "queue.dat"

// Queue Header Structure
typedef struct {
  uint32_t magic;             // Magic number to verify header integrity (0xGPSQ)
  uint32_t version;           // Version of the queue format
  uint32_t headIndex;         // Current head position in entries (write position)
  uint32_t tailIndex;         // Current tail position in entries (oldest data)
  uint32_t entryCount;        // Number of valid entries in the queue
  uint32_t totalEntriesEver;  // Total entries ever written
  uint32_t lastTimestamp;     // Unix timestamp of the most recent entry
  uint8_t reserved[36];       // Reserved for future use
} QueueHeader;

// Circular Queue structure
typedef struct {
  QueueHeader header;
  bool initialized;
} CircularQueue;

CircularQueue gpsQueue;

// Last logged second for 5-second interval
static int lastLoggedSecond = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */

// USB CDC related functions
void USB_CDC_Init(void);
void CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void processCommand(char* cmd);
void readQueueDataAndTransmitViaUSB(uint32_t sinceTimestamp);
// Circular Queue related functions
bool queue_init(void);
bool queue_enqueue(char* entry, uint32_t timestamp);
void formatDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds,
                   char* dateStr, char* timeStr);


void process_SD_card(void);
bool mcp7940n_init(void);
bool mcp7940n_read_time(void);
bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec);
bool mcp7940n_write_register(uint8_t reg, uint8_t value);
bool mcp7940n_read_register(uint8_t reg, uint8_t *value);

// GPS related functions
void process_gps_buffer(void);
void parse_gps_sentence(char* sentence);
float nmea_to_decimal_degrees(char* nmea_pos, char dir);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Format date and time strings for logging
void formatDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds,
                    char* dateStr, char* timeStr) {
  // Format date as "DDMMYY"
  sprintf(dateStr, "%02d%02d%02d", day, month, year);

  // Format time as "HHMMSS"
  sprintf(timeStr, "%02d%02d%02d", hours, minutes, seconds);
}

// Initialize the circular queue
bool queue_init(void) {
  FATFS fs;
  FIL indexFile;
  FRESULT fres;
  char buffer[64];
  int len;

  // Mount file system
  fres = f_mount(&fs, "", 1);
  if (fres != FR_OK) {
    len = sprintf(buffer, "Failed to mount filesystem: %d\r\n", fres);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    return false;
  }

  // Check if index file exists
  if (f_open(&indexFile, INDEX_FILENAME, FA_READ) == FR_OK) {
    // File exists, read header
    UINT bytesRead;
    fres = f_read(&indexFile, &gpsQueue.header, sizeof(QueueHeader), &bytesRead);
    f_close(&indexFile);

    if (fres != FR_OK || bytesRead != sizeof(QueueHeader)) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to read queue header\r\n", 29, HAL_MAX_DELAY);
      f_mount(NULL, "", 0);
      return false;
    }

    // Verify magic number
    if (gpsQueue.header.magic != 0x47505351) { // 'GPSQ'
      HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid header magic number\r\n", 29, HAL_MAX_DELAY);
      f_mount(NULL, "", 0);
      return false;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Queue loaded successfully\r\n", 26, HAL_MAX_DELAY);
  } else {
    // Create new queue header
    gpsQueue.header.magic = 0x47505351; // 'GPSQ'
    gpsQueue.header.version = 1;
    gpsQueue.header.headIndex = 0;
    gpsQueue.header.tailIndex = 0;
    gpsQueue.header.entryCount = 0;
    gpsQueue.header.totalEntriesEver = 0;
    gpsQueue.header.lastTimestamp = 0;

    // Create index file and save header
    fres = f_open(&indexFile, INDEX_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to create index file\r\n", 29, HAL_MAX_DELAY);
      f_mount(NULL, "", 0);
      return false;
    }

    UINT bytesWritten;
    fres = f_write(&indexFile, &gpsQueue.header, sizeof(QueueHeader), &bytesWritten);
    f_close(&indexFile);

    if (fres != FR_OK || bytesWritten != sizeof(QueueHeader)) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to write queue header\r\n", 30, HAL_MAX_DELAY);
      f_mount(NULL, "", 0);
      return false;
    }

    // Create data file
    FIL dataFile;
    fres = f_open(&dataFile, DATA_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to create data file\r\n", 28, HAL_MAX_DELAY);
      f_mount(NULL, "", 0);
      return false;
    }
    f_close(&dataFile);

    HAL_UART_Transmit(&huart2, (uint8_t*)"Queue initialized with new files\r\n", 34, HAL_MAX_DELAY);
  }

  gpsQueue.initialized = true;
  f_mount(NULL, "", 0);
  return true;
}




// Add a new entry to the queue
// Add a new entry to the queue
bool queue_enqueue(char* entry, uint32_t timestamp) {
  if (!gpsQueue.initialized) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Queue not initialized\r\n", 22, HAL_MAX_DELAY);
    return false;
  }

  FATFS fs;
  FIL dataFile;
  FRESULT fres;

  // Mount file system
  fres = f_mount(&fs, "", 1);
  if (fres != FR_OK) {
    char buffer[64];
    int len = sprintf(buffer, "Failed to mount filesystem: %d\r\n", fres);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
    return false;
  }

  // Open data file
  fres = f_open(&dataFile, DATA_FILENAME, FA_OPEN_ALWAYS | FA_WRITE);
  if (fres != FR_OK) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to open data file\r\n", 26, HAL_MAX_DELAY);
    f_mount(NULL, "", 0);
    return false;
  }

  // Check if queue is full before writing
  bool queueWasFull = (gpsQueue.header.entryCount >= QUEUE_CAPACITY);
  if (queueWasFull) {
    // If queue is full, move tail before writing
    gpsQueue.header.tailIndex = (gpsQueue.header.tailIndex + 1) % QUEUE_CAPACITY;
  }

  // Calculate position to write
  uint32_t position = (gpsQueue.header.headIndex % QUEUE_CAPACITY) * ENTRY_SIZE;

  // Seek to position
  fres = f_lseek(&dataFile, position);
  if (fres != FR_OK) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to seek in data file\r\n", 29, HAL_MAX_DELAY);
    f_close(&dataFile);
    f_mount(NULL, "", 0);
    return false;
  }

  // Write entry data
  size_t entryLen = strlen(entry);
  UINT bytesWritten;
  fres = f_write(&dataFile, entry, entryLen, &bytesWritten);

  // Fill remaining bytes with zeros
  if (bytesWritten < ENTRY_SIZE) {
    char zeros[ENTRY_SIZE - bytesWritten];
    memset(zeros, 0, ENTRY_SIZE - bytesWritten);
    f_write(&dataFile, zeros, ENTRY_SIZE - bytesWritten, &bytesWritten);
  }

  f_close(&dataFile);

  // Update header
  gpsQueue.header.headIndex = (gpsQueue.header.headIndex + 1) % QUEUE_CAPACITY;
  gpsQueue.header.totalEntriesEver++;
  gpsQueue.header.lastTimestamp = timestamp;

  // Update entry count if queue wasn't full
  if (!queueWasFull) {
    gpsQueue.header.entryCount++;
  }

  // Save updated header
  FIL indexFile;
  fres = f_open(&indexFile, INDEX_FILENAME, FA_OPEN_ALWAYS | FA_WRITE);
  if (fres != FR_OK) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to open index file for update\r\n", 38, HAL_MAX_DELAY);
    f_mount(NULL, "", 0);
    return false;
  }

  fres = f_write(&indexFile, &gpsQueue.header, sizeof(QueueHeader), &bytesWritten);
  f_close(&indexFile);

  // Unmount filesystem
  f_mount(NULL, "", 0);

  return (fres == FR_OK);
}



// Process USB commands
void processCommand(char* cmd) {
  // Remove any newline or carriage return
  char* newline = strchr(cmd, '\n');
  if (newline) *newline = '\0';

  newline = strchr(cmd, '\r');
  if (newline) *newline = '\0';

  // Define command constants
  const char* LOGGER_CMD = "GET_72HRS_DATA";
  const char* CMD_HEAD = "HEAD";
  const char* CMD_RST = "RST";
  const char* SET_SPEED_PREFIX = "SET_SPEED=";
  const char* SET_LIMP_PREFIX = "SET_LIMP=";
  const char* SET_TIME_PREFIX = "SET_TIME=";

  // Global variables for speed settings
  static int speedLimit = 80;
  static int limpSpeed = 40;

  char buffer[128];
  int len;

  // Debug info to UART2
  len = sprintf(buffer, "USB Command: %s\r\n", cmd);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

  // Process commands
  if (strcmp(cmd, LOGGER_CMD) == 0) {
    // Debug to UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)"Processing GET_72HRS_DATA command\r\n", 35, HAL_MAX_DELAY);

    // Get data from circular queue for last 72 hours
    len = sprintf(buffer, "Retrieving data from circular queue for last 72 hours...\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);

    // Calculate timestamp for 72 hours ago
    if (mcp7940n_read_time()) {
      uint32_t currentTimestamp =
          (year + 2000) * 365 * 24 * 60 * 60 +
          month * 30 * 24 * 60 * 60 +
          date * 24 * 60 * 60 +
          hours * 60 * 60 +
          minutes * 60 +
          seconds;

      uint32_t secondsIn72Hours = 3 * 24 * 60 * 60;
      uint32_t sinceTimestamp = (currentTimestamp > secondsIn72Hours) ?
                              currentTimestamp - secondsIn72Hours : 0;

      // Read and send entries
      readQueueDataAndTransmitViaUSB(sinceTimestamp);
    } else {
      len = sprintf(buffer, "ERROR: Could not read RTC time\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }
  }
  else if (strcmp(cmd, CMD_HEAD) == 0) {
    // Debug to UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)"Processing HEAD command\r\n", 24, HAL_MAX_DELAY);

    len = sprintf(buffer, "Current settings:\r\nSpeed limit: %d\r\nLimp speed: %d\r\n",
                  speedLimit, limpSpeed);
    CDC_Transmit_FS((uint8_t*)buffer, len);

    if (mcp7940n_read_time()) {
      len = sprintf(buffer, "Device time: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
             year, month, date, hours, minutes, seconds);
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }

    if (gpsQueue.initialized) {
      len = sprintf(buffer, "Queue entries: %lu of %d (%lu%%)\r\n",
             gpsQueue.header.entryCount,
             QUEUE_CAPACITY,
             (gpsQueue.header.entryCount * 100) / QUEUE_CAPACITY);
      CDC_Transmit_FS((uint8_t*)buffer, len);
    } else {
      len = sprintf(buffer, "Circular queue not initialized\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }
  }
  else if (strcmp(cmd, CMD_RST) == 0) {
    // Debug to UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)"Processing RST command\r\n", 24, HAL_MAX_DELAY);

    len = sprintf(buffer, "Resetting device...\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
    gps_valid_data = false;
    len = sprintf(buffer, "Reset complete\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
  }
  else if (strncmp(cmd, SET_SPEED_PREFIX, strlen(SET_SPEED_PREFIX)) == 0) {
    int newSpeed = atoi(cmd + strlen(SET_SPEED_PREFIX));

    // Debug to UART2
    len = sprintf(buffer, "Processing SET_SPEED command: %d\r\n", newSpeed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    if (newSpeed > 0 && newSpeed < 200) {
      speedLimit = newSpeed;
      len = sprintf(buffer, "Speed limit set to: %d\r\n", speedLimit);
      CDC_Transmit_FS((uint8_t*)buffer, len);
    } else {
      len = sprintf(buffer, "Invalid speed value\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }
  }
  else if (strncmp(cmd, SET_LIMP_PREFIX, strlen(SET_LIMP_PREFIX)) == 0) {
    int newLimpSpeed = atoi(cmd + strlen(SET_LIMP_PREFIX));

    // Debug to UART2
    len = sprintf(buffer, "Processing SET_LIMP command: %d\r\n", newLimpSpeed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    if (newLimpSpeed > 0 && newLimpSpeed < speedLimit) {
      limpSpeed = newLimpSpeed;
      len = sprintf(buffer, "Limp speed set to: %d\r\n", limpSpeed);
      CDC_Transmit_FS((uint8_t*)buffer, len);
    } else {
      len = sprintf(buffer, "Invalid limp speed value\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }
  }
  else if (strncmp(cmd, SET_TIME_PREFIX, strlen(SET_TIME_PREFIX)) == 0) {
    char* timeStr = cmd + strlen(SET_TIME_PREFIX);

    // Debug to UART2
    len = sprintf(buffer, "Processing SET_TIME command: %s\r\n", timeStr);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    int y, m, d, h, min, sec;

    if (sscanf(timeStr, "%d-%d-%d %d:%d:%d", &y, &m, &d, &h, &min, &sec) == 6) {
      // Set RTC time
      mcp7940n_set_time(y % 100, m, d, h, min, sec);
      len = sprintf(buffer, "Time set successfully\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    } else {
      len = sprintf(buffer, "Invalid time format. Use: YYYY-MM-DD HH:MM:SS\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }
  }
  else {
    // Debug to UART2
    len = sprintf(buffer, "Unknown command: %s\r\n", cmd);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    len = sprintf(buffer, "Unknown command\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
  }
}

// Read queue data and send over USB
void readQueueDataAndTransmitViaUSB(uint32_t sinceTimestamp) {
  if (!gpsQueue.initialized) {
    char buffer[64];
    int len = sprintf(buffer, "ERROR: Circular queue not initialized\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
    return;
  }

  FATFS fs;
  FIL dataFile;
  FRESULT fres;

  // Debug to UART2
  HAL_UART_Transmit(&huart2, (uint8_t*)"Reading queue data...\r\n", 22, HAL_MAX_DELAY);

  // Mount filesystem
  fres = f_mount(&fs, "", 1);
  if (fres != FR_OK) {
    char buffer[64];
    int len = sprintf(buffer, "ERROR: Failed to mount filesystem\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
    return;
  }

  // Open data file
  fres = f_open(&dataFile, DATA_FILENAME, FA_READ);
  if (fres != FR_OK) {
    char buffer[64];
    int len = sprintf(buffer, "ERROR: Failed to open data file\r\n");
    CDC_Transmit_FS((uint8_t*)buffer, len);
    f_mount(NULL, "", 0);
    return;
  }

  // Start from the tail (oldest data)
  uint32_t currentIndex = gpsQueue.header.tailIndex;
  char buffer[128];
  char entryBuffer[ENTRY_SIZE + 1]; // +1 for null terminator
  int entriesSent = 0;

  // Send headers
  int len = sprintf(buffer, "Retrieving entries from circular queue...\r\n");
  CDC_Transmit_FS((uint8_t*)buffer, len);

  // Debug to UART2
  len = sprintf(buffer, "Queue has %lu entries\r\n", gpsQueue.header.entryCount);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

  // Go through all entries in the queue
  for (uint32_t i = 0; i < gpsQueue.header.entryCount; i++) {
    // Calculate position in file
    uint32_t position = (currentIndex % QUEUE_CAPACITY) * ENTRY_SIZE;
    fres = f_lseek(&dataFile, position);

    if (fres != FR_OK) {
      len = sprintf(buffer, "ERROR: Failed to seek in data file\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
      break;
    }

    // Read entry
    UINT bytesRead;
    memset(entryBuffer, 0, ENTRY_SIZE + 1);
    fres = f_read(&dataFile, entryBuffer, ENTRY_SIZE, &bytesRead);

    if (fres != FR_OK || bytesRead != ENTRY_SIZE) {
      len = sprintf(buffer, "WARNING: Incomplete read from queue\r\n");
      CDC_Transmit_FS((uint8_t*)buffer, len);
    }

    entryBuffer[ENTRY_SIZE] = '\0'; // Ensure null termination

    // Process entry (extract timestamp from DDMMYY,HHMMSS format)
    if (strlen(entryBuffer) > 13) { // Minimum length for a valid entry
      int day = (entryBuffer[0] - '0') * 10 + (entryBuffer[1] - '0');
      int month = (entryBuffer[2] - '0') * 10 + (entryBuffer[3] - '0');
      int year = 2000 + (entryBuffer[4] - '0') * 10 + (entryBuffer[5] - '0');

      int hour = (entryBuffer[7] - '0') * 10 + (entryBuffer[8] - '0');
      int minute = (entryBuffer[9] - '0') * 10 + (entryBuffer[10] - '0');
      int second = (entryBuffer[11] - '0') * 10 + (entryBuffer[12] - '0');

      // Simple timestamp calculation
      uint32_t entryTimestamp =
          year * 365 * 24 * 60 * 60 +
          month * 30 * 24 * 60 * 60 +
          day * 24 * 60 * 60 +
          hour * 60 * 60 +
          minute * 60 +
          second;

      // Check if this entry is recent enough
      if (entryTimestamp >= sinceTimestamp) {
        // Send entry over USB
        len = sprintf(buffer, "%s\r\n", entryBuffer);
        CDC_Transmit_FS((uint8_t*)buffer, len);
        entriesSent++;

        // Debug every 50 entries
        if (entriesSent % 50 == 0) {
          len = sprintf(buffer, "Sent %d entries so far\r\n", entriesSent);
          HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }

        // Add small delay every few entries to prevent buffer overflow
        if (entriesSent % 20 == 0) {
          HAL_Delay(50);
        }
      }
    }

    // Move to next entry
    currentIndex = (currentIndex + 1) % QUEUE_CAPACITY;
  }

  f_close(&dataFile);
  f_mount(NULL, "", 0);

  // Send completion message
  len = sprintf(buffer, "Transfer complete: %d entries sent\r\n", entriesSent);
  CDC_Transmit_FS((uint8_t*)buffer, len);

  // Debug to UART2
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
















void process_SD_card(void)
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  FRESULT     fres;                 //Result after operations
  char        buf[100];
  int len=0;
  do
  {
    //Mount the SD Card
    fres = f_mount(&FatFs, "", 1);    //1=mount now
    if (fres != FR_OK)
    {
      //printf("No SD Card found : (%i)\r\n", fres);
      len = sprintf(buffer, "No SD Card found : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

      break;
    }
    //printf("SD Card Mounted Successfully!!!\r\n");
     HAL_UART_Transmit(&huart2, (uint8_t*)"SD Card Mounted Successfully!!!\r\n", 35, HAL_MAX_DELAY);
    //Read the SD Card Total size and Free Size
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;

    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

   // printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
    len = sprintf(buffer,"TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    //Open the file
    fres = f_open(&fil, "EmbeTronicX.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(fres != FR_OK)
    {
      //printf("File creation/open Error : (%i)\r\n", fres);
      len = sprintf(buffer,"File creation/open Error : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
      break;
    }

    //printf("Writing data!!!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)"Writing data!!!\r\n", 30, HAL_MAX_DELAY);
    //write the data
    f_puts("Welcome to EmbeTronicX", &fil);

    //close your file
    f_close(&fil);

    //Open the file
    fres = f_open(&fil, "EmbeTronicX.txt", FA_READ);
    if(fres != FR_OK)
    {
      //printf("File opening Error : (%i)\r\n", fres);
      len = sprintf(buffer,"File opening Error : (%i)\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
      break;
    }

    //read the data
    f_gets(buf, sizeof(buf), &fil);

   // printf("Read Data : %s\n", buf);
    len =  sprintf(buffer,"Read Data : %s\n", buf);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    //close your file
    f_close(&fil);
   // printf("Closing File!!!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)"Closing File!!!\r\n", 30, HAL_MAX_DELAY);
#if 0
    //Delete the file.
    fres = f_unlink(EmbeTronicX.txt);
    if (fres != FR_OK)
    {
      printf("Cannot able to delete the file\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)"Cannot able to delete the file\n", 30, HAL_MAX_DELAY);
    }
#endif
  } while(false);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);
  printf("SD Card Unmounted Successfully!!!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)"SD Card Unmounted Successfully!!!\r\n", 36, HAL_MAX_DELAY);
}

// Process GPS data from DMA buffer
void process_gps_buffer(void)
{
  static uint16_t last_pos = 0;
  uint16_t pos = GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

  // Only process if position changed
  if (pos == last_pos) return;

  // Handle buffer wraparound
  if (pos < last_pos) {
    // Process from last_pos to end of buffer
    for (uint16_t i = last_pos; i < GPS_BUFFER_SIZE; i++) {
      if (gps_dma_buffer[i] == '$') {
        // Start of new NMEA sentence
        gps_line_index = 0;
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];
      }
      else if (gps_line_index > 0 && gps_line_index < sizeof(gps_line_buffer) - 1) {
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];

        // Process complete line on CR or LF
        if (gps_dma_buffer[i] == '\n' || gps_dma_buffer[i] == '\r') {
          gps_line_buffer[gps_line_index] = '\0';
          parse_gps_sentence((char*)gps_line_buffer);
          gps_line_index = 0;
        }
      }
      else if (gps_line_index >= sizeof(gps_line_buffer) - 1) {
        // Buffer overflow protection
        gps_line_index = 0;
      }
    }

    // Process from start to current position
    for (uint16_t i = 0; i < pos; i++) {
      if (gps_dma_buffer[i] == '$') {
        gps_line_index = 0;
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];
      }
      else if (gps_line_index > 0 && gps_line_index < sizeof(gps_line_buffer) - 1) {
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];

        if (gps_dma_buffer[i] == '\n' || gps_dma_buffer[i] == '\r') {
          gps_line_buffer[gps_line_index] = '\0';
          parse_gps_sentence((char*)gps_line_buffer);
          gps_line_index = 0;
        }
      }
      else if (gps_line_index >= sizeof(gps_line_buffer) - 1) {
        gps_line_index = 0;
      }
    }
  } else {
    // Process from last_pos to current position
    for (uint16_t i = last_pos; i < pos; i++) {
      if (gps_dma_buffer[i] == '$') {
        gps_line_index = 0;
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];
      }
      else if (gps_line_index > 0 && gps_line_index < sizeof(gps_line_buffer) - 1) {
        gps_line_buffer[gps_line_index++] = gps_dma_buffer[i];

        if (gps_dma_buffer[i] == '\n' || gps_dma_buffer[i] == '\r') {
          gps_line_buffer[gps_line_index] = '\0';

          // Debug output of raw NMEA
         // HAL_UART_Transmit(&huart2, (uint8_t*)"NMEA: ", 6, HAL_MAX_DELAY);
         // HAL_UART_Transmit(&huart2, (uint8_t*)gps_line_buffer, gps_line_index, HAL_MAX_DELAY);
         // HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

          parse_gps_sentence((char*)gps_line_buffer);
          gps_line_index = 0;
        }
      }
      else if (gps_line_index >= sizeof(gps_line_buffer) - 1) {
        gps_line_index = 0;
      }
    }
  }

  last_pos = pos;
}

// Fixed NMEA position format to decimal degrees conversion
float nmea_to_decimal_degrees(char* nmea_pos, char dir)
{
  // Validate input
  if (!nmea_pos || strlen(nmea_pos) < 4) return 0.0;

  // Copy string to work with it safely
  char pos_str[16];
  strncpy(pos_str, nmea_pos, 15);
  pos_str[15] = '\0';

  // Determine degree length based on N/S or E/W
  int deg_len = (dir == 'N' || dir == 'S') ? 2 : 3;

  // Extract degrees part
  pos_str[deg_len] = '\0'; // Temporarily terminate string to extract degrees
  float degrees = 0;

  // Parse degrees manually
  for (int i = 0; pos_str[i] != '\0'; i++) {
    if (pos_str[i] >= '0' && pos_str[i] <= '9') {
      degrees = degrees * 10 + (pos_str[i] - '0');
    }
  }

  // Extract minutes part
  float minutes = 0;
  char *min_ptr = nmea_pos + deg_len;
  int decimal_found = 0;
  float decimal_place = 0.1f;

  for (int i = 0; min_ptr[i] != '\0'; i++) {
    if (min_ptr[i] == '.') {
      decimal_found = 1;
    } else if (min_ptr[i] >= '0' && min_ptr[i] <= '9') {
      if (!decimal_found) {
        minutes = minutes * 10 + (min_ptr[i] - '0');
      } else {
        minutes += (min_ptr[i] - '0') * decimal_place;
        decimal_place *= 0.1f;
      }
    }
  }

  // Calculate decimal degrees
  float decimal_degrees = degrees + (minutes / 60.0f);

  // Apply sign based on hemisphere
  if (dir == 'S' || dir == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

// NMEA sentence parser - Updated for LC86G module
void parse_gps_sentence(char* sentence)
{
  // Skip invalid sentences or too short
  if (sentence[0] != '$' || strlen(sentence) < 10) return;

  // Check sentence type
  bool is_gga = (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0);
  bool is_rmc = (strncmp(sentence, "$GNRMC", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0);
  bool is_gll = (strncmp(sentence, "$GNGLL", 6) == 0 || strncmp(sentence, "$GPGLL", 6) == 0);
  bool is_vtg = (strncmp(sentence, "$GNVTG", 6) == 0 || strncmp(sentence, "$GPVTG", 6) == 0);
  bool is_antenna = (strncmp(sentence, "$PQTMANTENNASTATUS", 18) == 0);

  // Skip if not a position or speed sentence
  if (!is_gga && !is_rmc && !is_gll && !is_vtg && !is_antenna) return;

  // Split the sentence into parts
  char *parts[20] = {NULL};
  int part_count = 0;

  // First field is the sentence type
  parts[part_count++] = sentence;

  // Split by commas
  char *ptr = sentence;
  while (*ptr && part_count < 20) {
    if (*ptr == ',') {
      *ptr = '\0';
      parts[part_count++] = ptr + 1;
    } else if (*ptr == '*') {
      *ptr = '\0';
      break;
    }
    ptr++;
  }

  // Process based on sentence type
  if (is_gga && part_count >= 15) {
    // GGA: $--GGA,time,lat,N/S,lon,E/W,quality,satellites,...

    // Only process if we have a valid fix (quality > 0)
    if (part_count > 6 && parts[6] && *parts[6] > '0') {
      // Store timestamp
      if (parts[1] && strlen(parts[1]) >= 6) {
        strncpy(gps_timestamp, parts[1], 9);
        gps_timestamp[9] = '\0';
      }

      // Process fix quality
      gps_fix_quality = 0;
      for (int i = 0; parts[6][i] != '\0'; i++) {
        if (parts[6][i] >= '0' && parts[6][i] <= '9') {
          gps_fix_quality = gps_fix_quality * 10 + (parts[6][i] - '0');
        }
      }

      // Process satellites
      if (part_count > 7 && parts[7] && *parts[7]) {
        gps_satellites = 0;
        for (int i = 0; parts[7][i] != '\0'; i++) {
          if (parts[7][i] >= '0' && parts[7][i] <= '9') {
            gps_satellites = gps_satellites * 10 + (parts[7][i] - '0');
          }
        }
      }

      // Process position - with careful validation
      if (part_count > 5 && parts[2] && *parts[2] && parts[4] && *parts[4]) {
        // Validate latitude format (ddmm.mmmm)
        if (strlen(parts[2]) >= 4 && parts[3] && (*parts[3] == 'N' || *parts[3] == 'S')) {
          gps_latitude = nmea_to_decimal_degrees(parts[2], *parts[3]);
        }

        // Validate longitude format (dddmm.mmmm)
        if (strlen(parts[4]) >= 5 && parts[5] && (*parts[5] == 'E' || *parts[5] == 'W')) {
          gps_longitude = nmea_to_decimal_degrees(parts[4], *parts[5]);
        }

        gps_valid_data = true;
      }
    } else {
      gps_fix_quality = 0;
      gps_valid_data = false;
    }
  }
  else if (is_rmc && part_count >= 10) {
    // RMC: $--RMC,time,status,lat,N/S,lon,E/W,speed,course,date,magnetic variation,E/W,mode

    // Check if data is valid
    bool valid_status = (parts[2] && *parts[2] == 'A');

    if (valid_status) {
      // Store timestamp
      if (parts[1] && strlen(parts[1]) >= 6) {
        strncpy(gps_timestamp, parts[1], 9);
        gps_timestamp[9] = '\0';
      }

      // Process date
      if (part_count > 9 && parts[9] && strlen(parts[9]) >= 6) {
        strncpy(gps_date, parts[9], 6);
        gps_date[6] = '\0';
      }

      // Process speed
      if (part_count > 7 && parts[7] && *parts[7]) {
        float knots = 0.0f;
        int decimal_found = 0;
        float decimal_place = 0.1f;

        for (int i = 0; parts[7][i] != '\0'; i++) {
          if (parts[7][i] == '.') {
            decimal_found = 1;
          } else if (parts[7][i] >= '0' && parts[7][i] <= '9') {
            if (!decimal_found) {
              knots = knots * 10.0f + (parts[7][i] - '0');
            } else {
              knots += (parts[7][i] - '0') * decimal_place;
              decimal_place *= 0.1f;
            }
          }
        }

        gps_speed_kmh = knots * 1.852f; // Convert knots to km/h
      }

      // Process position - if not already set by GGA
      if (part_count > 5 && parts[3] && *parts[3] && parts[5] && *parts[5]) {
        // Validate latitude format
        if (strlen(parts[3]) >= 4 && parts[4] && (*parts[4] == 'N' || *parts[4] == 'S')) {
          gps_latitude = nmea_to_decimal_degrees(parts[3], *parts[4]);
        }

        // Validate longitude format
        if (strlen(parts[5]) >= 5 && parts[6] && (*parts[6] == 'E' || *parts[6] == 'W')) {
          gps_longitude = nmea_to_decimal_degrees(parts[5], *parts[6]);
        }

        gps_valid_data = true;
      }
    }
  }
  else if (is_gll && part_count >= 7) {
    // GLL: $--GLL,lat,N/S,lon,E/W,time,status

    // Check if the fix is valid
    bool valid_status = (part_count > 6 && parts[6] && *parts[6] == 'A');

    if (valid_status) {
      // Store timestamp
      if (part_count > 5 && parts[5] && strlen(parts[5]) >= 6) {
        strncpy(gps_timestamp, parts[5], 9);
        gps_timestamp[9] = '\0';
      }

      // Process position - with careful validation
      if (parts[1] && *parts[1] && parts[3] && *parts[3]) {
        // Validate latitude format
        if (strlen(parts[1]) >= 4 && parts[2] && (*parts[2] == 'N' || *parts[2] == 'S')) {
          gps_latitude = nmea_to_decimal_degrees(parts[1], *parts[2]);
        }

        // Validate longitude format
        if (strlen(parts[3]) >= 5 && parts[4] && (*parts[4] == 'E' || *parts[4] == 'W')) {
          gps_longitude = nmea_to_decimal_degrees(parts[3], *parts[4]);
        }

        gps_valid_data = true;
      }
    }
  }
  else if (is_vtg && part_count >= 8) {
    // VTG: $--VTG,course,T,course,M,speed,N,speed,K,mode

    // Process speed in km/h if available
    if (part_count > 7 && parts[7] && *parts[7]) {
      float kmh = 0.0f;
      int decimal_found = 0;
      float decimal_place = 0.1f;

      for (int i = 0; parts[7][i] != '\0'; i++) {
        if (parts[7][i] == '.') {
          decimal_found = 1;
        } else if (parts[7][i] >= '0' && parts[7][i] <= '9') {
          if (!decimal_found) {
            kmh = kmh * 10.0f + (parts[7][i] - '0');
          } else {
            kmh += (parts[7][i] - '0') * decimal_place;
            decimal_place *= 0.1f;
          }
        }
      }

      gps_speed_kmh = kmh; // Already in km/h
    }
  }
  else if (is_antenna && part_count >= 5) {
    // PQTMANTENNASTATUS: Proprietary message for antenna status
    gps_antenna_status = true;
  }
}

// DMA callback for UART reception
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    // Restart DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_buffer, GPS_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  // Change USART2 baud rate to 9600
  huart2.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure UART3 for LC86G GPS at 9600 baud
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  // Clear GPS buffers
  memset(gps_dma_buffer, 0, GPS_BUFFER_SIZE);
  memset(gps_line_buffer, 0, sizeof(gps_line_buffer));

  // Send initialization message
  HAL_UART_Transmit(&huart2, (uint8_t*)"System initializing...\r\n", 23, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing MCP7940N RTC...\r\n", 30, HAL_MAX_DELAY);

  //HAL_UART_Transmit(&huart2, (uint8_t*)"RTC INITIALIZING....\R \N",23,HAL_MAX_DELAY);

  // Initialize MCP7940N RTC
  if (mcp7940n_init())
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"RTC initialized successfully\r\n", 29, HAL_MAX_DELAY);

    // Set initial time: 2023/04/09 14:30:00
/*
    if (mcp7940n_set_time(25, 4, 22, 16, 30, 0))
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Time set successfully\r\n", 23, HAL_MAX_DELAY);
    }
    else
    {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to set time\r\n", 20, HAL_MAX_DELAY);
    }
*/

  }
  else
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to initialize RTC\r\n", 26, HAL_MAX_DELAY);
  }


  //CIRCULAR QUEUE INITIALIZING
  HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing circular queue...\r\n", 32, HAL_MAX_DELAY);
  if (queue_init()) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Circular queue initialized successfully\r\n", 41, HAL_MAX_DELAY);
  } else {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Failed to initialize circular queue\r\n", 37, HAL_MAX_DELAY);
  }

  // Debug message for GPS setup
  HAL_UART_Transmit(&huart2, (uint8_t*)"Setting up LC86G GPS module...\r\n", 32, HAL_MAX_DELAY);

  // Start DMA reception for UART3
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_buffer, GPS_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  process_SD_card();

  // Debug message for main loop
  HAL_UART_Transmit(&huart2, (uint8_t*)"Starting main loop...\r\n", 22, HAL_MAX_DELAY);

  uint32_t last_rtc_update = HAL_GetTick();
  uint32_t last_gps_update = HAL_GetTick();

  while (1)
  {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      // Read current time from RTC every 5 seconds
      if (HAL_GetTick() - last_rtc_update > 5000)
      {
          if (mcp7940n_read_time())
          {
              // Format the date and time string
              sprintf(buffer, "RTC Time: 20%02d/%02d/%02d %02d:%02d:%02d\r\n",
                      year, month, date, hours, minutes, seconds);
              // Send to UART2
              HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
          }
          else
          {
              HAL_UART_Transmit(&huart2, (uint8_t*)"Error reading RTC\r\n", 19, HAL_MAX_DELAY);
          }

          last_rtc_update = HAL_GetTick();
      }

      // Process GPS data
      process_gps_buffer();

      // Output GPS data every second
      if (HAL_GetTick() - last_gps_update > 1000)
      {
          char gps_msg[128];

          if (gps_valid_data)
          {
              // Format with correct decimal places and order
              sprintf(gps_msg, "%s,%s,%.6f%c,%.6f%c,%.1f\r\n",
                      gps_date,                          // Date (DDMMYY)
                      gps_timestamp,                     // Time (HHMMSS.SSS)
                      fabsf(gps_longitude),              // Longitude value (6 decimal places)
                      gps_longitude >= 0 ? 'E' : 'W',    // E/W indicator
                      fabsf(gps_latitude),               // Latitude value (6 decimal places)
                      gps_latitude >= 0 ? 'N' : 'S',     // N/S indicator
                      gps_speed_kmh);                    // Speed in km/h

              HAL_UART_Transmit(&huart2, (uint8_t*)gps_msg, strlen(gps_msg), HAL_MAX_DELAY);
          }
          else
          {
              // No valid fix yet
              HAL_UART_Transmit(&huart2, (uint8_t*)"gps signal lost\r\n", 17, HAL_MAX_DELAY);
          }

          last_gps_update = HAL_GetTick();
      }

      // Check RTC time for 5-second precise logging
      if (mcp7940n_read_time())
      {
          // Check if it's exactly a multiple of 5 seconds
          if ((seconds % 5) == 0 && seconds != lastLoggedSecond)
          {
              lastLoggedSecond = seconds;

              // Format date and time for logging
              char rtcDate[7]; // DDMMYY
              char rtcTime[7]; // HHMMSS
              formatDateTime(date, month, year, hours, minutes, seconds, rtcDate, rtcTime);

              // Prepare log line
              char logLine[64];

              if (gps_valid_data)
              {
                  char speedStr[10];
                  sprintf(speedStr, "%.1f", gps_speed_kmh);

                  sprintf(logLine, "%s,%s,%.6f%c,%.6f%c,%s",
                          rtcDate, rtcTime,
                          fabsf(gps_longitude), gps_longitude >= 0 ? 'E' : 'W',
                          fabsf(gps_latitude), gps_latitude >= 0 ? 'N' : 'S',
                          speedStr);
              }
              else
              {
                  // No valid GPS data - use "SL" (Signal Lost) markers
                  sprintf(logLine, "%s,%s,SL,SL,SL", rtcDate, rtcTime);
              }

              // Log to circular queue
              if (gpsQueue.initialized)
              {
                  // Simple timestamp calculation
                  uint32_t timestamp =
                      (year + 2000) * 365 * 24 * 60 * 60 +
                      month * 30 * 24 * 60 * 60 +
                      date * 24 * 60 * 60 +
                      hours * 60 * 60 +
                      minutes * 60 +
                      seconds;

                  bool logSuccess = queue_enqueue(logLine, timestamp);

                  // Debug message to UART2
                  char debug[128];
                  int len = sprintf(debug, "5s log: %s (%s)\r\n", logLine, logSuccess ? "Success" : "Failed");
                  HAL_UART_Transmit(&huart2, (uint8_t*)debug, len, HAL_MAX_DELAY);
              }
          }
      }

      // Short delay for system stability
      HAL_Delay(10);
  }
  /* USER CODE END 3 */
  //***********************************************************MAIN-END*************************************
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//***********************************************************MCP7940N-START*************************************
// MCP7940N I2C address (7-bit address)
#define MCP7940N_ADDRESS 0x6F

// MCP7940N registers
#define MCP7940N_REG_SECONDS 0x00
#define MCP7940N_REG_MINUTES 0x01
#define MCP7940N_REG_HOURS   0x02
#define MCP7940N_REG_WKDAY   0x03
#define MCP7940N_REG_DATE    0x04
#define MCP7940N_REG_MONTH   0x05
#define MCP7940N_REG_YEAR    0x06
#define MCP7940N_REG_CONTROL 0x07
#define MCP7940N_REG_OSCTRIM 0x08

// Storage for date/time
uint8_t seconds, minutes, hours, day, date, month, year;

// Write a register on the MCP7940N
bool mcp7940n_write_register(uint8_t reg, uint8_t value)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = value;

  if (HAL_I2C_Master_Transmit(&hi2c1, MCP7940N_ADDRESS << 1, data, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }
  return true;
}

// Read a register from the MCP7940N
bool mcp7940n_read_register(uint8_t reg, uint8_t *value)
{
  if (HAL_I2C_Master_Transmit(&hi2c1, MCP7940N_ADDRESS << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Master_Receive(&hi2c1, MCP7940N_ADDRESS << 1 | 1, value, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return false;
  }

  return true;
}

// Read time from MCP7940N
bool mcp7940n_read_time()
{
  uint8_t value;

  // Read seconds
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) return false;
  // Strip off the ST bit (bit 7)
  seconds = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  // Read minutes
  if (!mcp7940n_read_register(MCP7940N_REG_MINUTES, &value)) return false;
  minutes = (value & 0x0F) + ((value & 0x70) >> 4) * 10;

  // Read hours (assuming 24-hour mode)
  if (!mcp7940n_read_register(MCP7940N_REG_HOURS, &value)) return false;
  hours = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  // Read weekday
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) return false;
  day = value & 0x07;  // Only lower 3 bits represent day of week

  // Read date
  if (!mcp7940n_read_register(MCP7940N_REG_DATE, &value)) return false;
  date = (value & 0x0F) + ((value & 0x30) >> 4) * 10;

  // Read month
  if (!mcp7940n_read_register(MCP7940N_REG_MONTH, &value)) return false;
  month = (value & 0x0F) + ((value & 0x10) >> 4) * 10;

  // Read year
  if (!mcp7940n_read_register(MCP7940N_REG_YEAR, &value)) return false;
  year = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;

  return true;
}

// Set time on MCP7940N
bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec)
{
  // Convert decimal to BCD
  uint8_t sec_bcd = ((sec / 10) << 4) | (sec % 10);
  uint8_t min_bcd = ((min / 10) << 4) | (min % 10);
  uint8_t hour_bcd = ((h / 10) << 4) | (h % 10);
  uint8_t date_bcd = ((d / 10) << 4) | (d % 10);
  uint8_t month_bcd = ((m / 10) << 4) | (m % 10);
  uint8_t year_bcd = ((y / 10) << 4) | (y % 10);
  uint8_t wkday = 1;  // Default to Monday

  // For seconds, set the ST bit (bit 7) to enable oscillator
  sec_bcd |= 0x80;

  // Write time/date registers
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, sec_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MINUTES, min_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_HOURS, hour_bcd)) return false;
  // Set the VBATEN bit (bit 3) in RTCWKDAY register to enable battery backup
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, wkday | 0x08)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_DATE, date_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MONTH, month_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_YEAR, year_bcd)) return false;

  return true;
}

// Initialize the MCP7940N RTC
bool mcp7940n_init()
{
  // Test communication
  uint8_t value;
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }

  // Enable oscillator by setting ST bit (bit 7) in seconds register
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) {
    return false;
  }
  value |= 0x80;  // Set ST bit
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, value)) {
    return false;
  }

  // Enable battery backup by setting VBATEN bit (bit 3) in RTCWKDAY register
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }
  value |= 0x08;  // Set VBATEN bit
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, value)) {
    return false;
  }

  // Set control register (basic configuration)
  return mcp7940n_write_register(MCP7940N_REG_CONTROL, 0x00);
}

//***********************************************************MCP7940N-END*************************************

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
