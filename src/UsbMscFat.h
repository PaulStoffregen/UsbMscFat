/**
 * Copyright (c) 2017-2020 Warren Watson
 * This file is part of the SdFat library for use with MSC.
 * 
 * Copyright (c) 2011-2019 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * Modified 2020 for use with SdFat and MSC. By Warren Watson.
 * 
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */



#include <Arduino.h>
#include <SdFat.h>
#include <USBHost_t36.h>
#include "mscSenseKeyList.h"

#ifndef UsbMscFat_h
#define UsbMscFat_h

#ifndef USBmscInfo_h
#define USBmscInfo_h

const char *decodeSenseKey(uint8_t senseKey);
const char *decodeAscAscq(uint8_t asc, uint8_t ascq);

void printMscAscError(print_t* pr, msController *pDrive);

//-----------------------------------------------------------------------------

inline uint32_t USBmscCapacity(msController *pDrv) {
	return (pDrv->msDriveInfo.capacity.Blocks); 
}

#endif  // USBmscInfo_h



#ifndef USBmscInterface_h
#define USBmscInterface_h
/**
 * \class USBmscInterface
 * \brief Abstract interface for a USB Mass Storage Device.
 */
class USBmscInterface : public FsBlockDeviceInterface {
 public:
  /** \return error code. */
  virtual uint8_t errorCode() const = 0;
  /** \return error data. */
  virtual uint32_t errorData() const = 0;
  /** \return true if USB is busy. */
  virtual bool isBusy() = 0;
  /** \return true if USB read is busy. */
  virtual bool isBusyRead();
  /** \return true if USB write is busy. */
  virtual bool isBusyWrite();
    /** Read a MSC USB drive's info.
   * \return true for success or false for failure.
   */
  virtual bool readUSBDriveInfo(msDriveInfo_t * driveInfo) = 0;
  /**
   * Determine the size of a USB Mass Storage Device.
   *
   * \return The number of 512 byte data sectors in the USB device
   *         or zero if an error occurs.
   */
  virtual uint32_t sectorCount() = 0;
  /** \return USB drive status. */
  virtual uint32_t status() {return 0XFFFFFFFF;}

  virtual bool readSectorsWithCB(uint32_t sector, size_t ns, void (*callback)(uint32_t, uint8_t *), uint32_t token) = 0;

};
#endif  // USBmscInterface_h

#ifndef USBmscDevice_h
#define USBmscDevice_h
/**
 * \class USBMSCDevice
 * \brief Raw USB Drive accesss.
 */
class USBMSCDevice : public USBmscInterface {
 public:
  /** Initialize the USB MSC device.
   * \param[in] Pointer to an instance of msc.
   * \return true for success or false for failure.
   */
  bool begin(msController *pDrive);
  uint32_t sectorCount();
  /**
   * \return code for the last error. See USBmscInfo.h for a list of error codes.
   */
  uint8_t errorCode() const;
  /** \return error data for last error. */
  uint32_t errorData() const;
  /** \return error line for last error. Tmp function for debug. */
  uint32_t errorLine() const;
  /**
   * Check for busy with CMD13.
   *
   * \return true if busy else false.
   */
  bool isBusy();
  /** Check for busy with MSC read operation
   *
   * \return true if busy else false.
   */
  bool isBusyRead();
  /** Check for busy with MSC read operation
   *
   * \return true if busy else false.
   */
  bool isBusyWrite();
  /**
   * Read a USB drive's information. This contains the drive's identification
   * information such as Manufacturer ID, Product name, Product serial
   * number and Manufacturing date pluse more.
   *
   * \param[out]  msDriveInfo_t pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readUSBDriveInfo(msDriveInfo_t * driveInfo);
  /**
   * Read a 512 byte sector from an USB MSC drive.
   *
   * \param[in] sector Logical sector to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
  bool readSector(uint32_t sector, uint8_t* dst);
  /**
   * Read multiple 512 byte sectors from an USB MSC drive.
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
  bool readSectors(uint32_t sector, uint8_t* dst, size_t ns);
  /** \return USB MSC drive status. */
  uint32_t status();
  /** \return success if sync successful. Not for user apps. */
  bool syncDevice();
  /**
   * Writes a 512 byte sector to an USB MSC drive.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
  bool writeSector(uint32_t sector, const uint8_t* src);
  /**
   * Write multiple 512 byte sectors to an USB MSC drive.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] ns Number of sectors to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
  bool writeSectors(uint32_t sector, const uint8_t* src, size_t ns);

  /**
   * Read multiple 512 byte sectors from an USB MSC drive, using 
   * a callback per sector
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[in] callback function to call for each sector read.
   * \return true for success or false for failure.
   */
  bool readSectorsWithCB(uint32_t sector, size_t ns, void (*callback)(uint32_t, uint8_t *), uint32_t token);


private:
  msController *thisDrive;
};
#endif  // USBmscDevice_h




/** MSCFat version */
#define MSC_FAT_VERSION "1.0.0"

//==============================================================================

class UsbFs : public FsVolume {
 public:
  //----------------------------------------------------------------------------
  /** Initialize USB drive and file system.
   *
   * \param[in] msController drive.
   * \return true for success or false for failure.
   */
  bool begin(msController *pDrive, bool setCwv = true, uint8_t part = 1) {
    // Serial.printf("UsbFs::begin called %x %x %d\n", (uint32_t)pDrive, setCwv, part);
    device.begin(pDrive);
    thisMscDrive = pDrive;
    if (device.errorCode() != 0) return false;
    // Serial.println("    After usbDriveBegin");
    return FsVolume::begin(&device, setCwv, part);
  }
  //---------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] pr Print destination.
   */
  void errorHalt(print_t* pr) {
    if (mscErrorCode()) {
      pr->print(F("mscError: 0X"));
      pr->print(mscErrorCode(), HEX);
      pr->print(F(",0X"));
      pr->println(mscErrorData(), HEX);
    } else if (!FsVolume::fatType()) {
      pr->println(F("Check USB drive format."));
    }
    while (1) ; //SysCall::halt();
  }
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void errorHalt(print_t* pr, const char* msg) {
    pr->print(F("error: "));
    pr->println(msg);
    errorHalt(pr);
  }
  //----------------------------------------------------------------------------
  /** %Print msg and halt.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void errorHalt(print_t* pr, const __FlashStringHelper* msg) {
    pr->print(F("error: "));
    pr->println(msg);
    errorHalt(pr);
  }
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] pr Print destination.
   */
  void initErrorHalt(print_t* pr) {
    initErrorPrint(pr);
    while (1) ; // SysCall::halt();
  }
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void initErrorHalt(print_t* pr, const char* msg) {
    pr->println(msg);
    initErrorHalt(pr);
  }
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void initErrorHalt(Print* pr, const __FlashStringHelper* msg) {
    pr->println(msg);
    initErrorHalt(pr);
  }
  //----------------------------------------------------------------------------
  /** Print error details after begin() fails.
   *
   * \param[in] pr Print destination.
   */
  void initErrorPrint(Print* pr) {
    pr->println(F("begin() failed"));
    if (mscErrorCode()) {
      pr->println(F("Do not reformat the USB drive."));
      if (mscErrorCode() == MS_NO_MEDIA_ERR) {
        pr->println(F("Is USB drive connected?"));
      }
    }
    errorPrint(pr);
  }
  //----------------------------------------------------------------------------
  /** %Print volume FAT/exFAT type.
   *
   * \param[in] pr Print destination.
   */
  void printFatType(print_t* pr) {
    if (FsVolume::fatType() == FAT_TYPE_EXFAT) {
      pr->print(F("exFAT"));
    } else {
      pr->print(F("FAT"));
      pr->print(FsVolume::fatType());
    }
  }
  //----------------------------------------------------------------------------
  /** %Print USB drive errorCode and errorData.
   *
   * \param[in] pr Print destination.
   */
  void errorPrint(print_t* pr) {
    if (mscErrorCode()) {
      pr->print(F("mscError: 0X"));
      pr->println(mscErrorCode(), HEX);
      // pr->print(F(",0X"));
      // pr->println(mscErrorData(), HEX);
    } else if (!FsVolume::fatType()) {
      pr->println(F("Check USB drive format."));
    }
  }
  //----------------------------------------------------------------------------
  /** %Print msg, any USB drive error code.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void errorPrint(print_t* pr, char const* msg) {
    pr->print(F("error: "));
    pr->println(msg);
    errorPrint(pr);
  }

  /** %Print msg, any USB drive error code.
   *
   * \param[in] pr Print destination.
   * \param[in] msg Message to print.
   */
  void errorPrint(Print* pr, const __FlashStringHelper* msg) {
    pr->print(F("error: "));
    pr->println(msg);
    errorPrint(pr);
  }
  //----------------------------------------------------------------------------
  /** %Print error info and return.
   *
   * \param[in] pr Print destination.
   */
  void printMscError(print_t* pr) {
    if (mscErrorCode()) {
      if (mscErrorCode() == 0x28) {
        pr->println(F("No USB drive detected, plugged in?"));
      }
      pr->print(F("USB drive error: "));
      pr->print(F("0x"));
      pr->print(mscErrorCode(), HEX);
      pr->print(F(",0x"));
      pr->print(mscErrorData(), HEX);
      printMscAscError(pr, thisMscDrive);
    } else if (!FsVolume::fatType()) {
      pr->println(F("Check USB drive format."));
    }
  }
  //----------------------------------------------------------------------------
  /** \return USB drive error code. */
  uint8_t mscErrorCode() { return device.errorCode(); }
  //----------------------------------------------------------------------------
  /** \return SD card error data. */
  uint8_t mscErrorData() { return device.errorData(); }
  //----------------------------------------------------------------------------
  /** \return pointer to base volume */
  FsVolume * vol() { return this; }

  //----------------------------------------------------------------------------
  /** Initialize file system after call to cardBegin.
   *
   * \return true for success or false for failure.
   */
  bool volumeBegin() { return FsVolume::begin(&device); } // is this redundant?
#if 0
  bool format(print_t* pr = nullptr) {
    static_assert(sizeof(m_volMem) >= 512, "m_volMem too small");
    uint32_t sectorCount = device.sectorCount();
    if (sectorCount == 0) {
      return false;
    }
    end();
    if (sectorCount > 67108864) {
      ExFatFormatter fmt;
      return fmt.format(&device, reinterpret_cast<uint8_t*>(m_volMem), pr);
    } else {
      FatFormatter fmt;
      return fmt.format(&device, reinterpret_cast<uint8_t*>(m_volMem), pr);
    }
  }
#endif

#if ENABLE_ARDUINO_SERIAL
  /** Print error details after begin() fails. */
  void initErrorPrint() {
    initErrorPrint(&Serial);
  }
  //----------------------------------------------------------------------------
  /** %Print msg to Serial and halt.
   *
   * \param[in] msg Message to print.
   */
  void errorHalt(const __FlashStringHelper* msg) {
    errorHalt(&Serial, msg);
  }
  //----------------------------------------------------------------------------
  /** %Print error info to Serial and halt. */
  void errorHalt() {errorHalt(&Serial);}
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] msg Message to print.
   */
  void errorHalt(const char* msg) {errorHalt(&Serial, msg);}
  //----------------------------------------------------------------------------
  /** %Print error info and halt. */
  void initErrorHalt() {initErrorHalt(&Serial);}
  //----------------------------------------------------------------------------
  /** %Print msg, any SD error code.
   *
   * \param[in] msg Message to print.
   */
  void errorPrint(const char* msg) {errorPrint(&Serial, msg);}
   /** %Print msg, any SD error code.
   *
   * \param[in] msg Message to print.
   */
  void errorPrint(const __FlashStringHelper* msg) {errorPrint(&Serial, msg);}
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] msg Message to print.
   */
  void initErrorHalt(const char* msg) {initErrorHalt(&Serial, msg);}
  //----------------------------------------------------------------------------
  /** %Print error info and halt.
   *
   * \param[in] msg Message to print.
   */
  void initErrorHalt(const __FlashStringHelper* msg) {
    initErrorHalt(&Serial, msg);
  }
#endif  // ENABLE_ARDUINO_SERIAL
  //----------------------------------------------------------------------------
 private:
  USBMSCDevice device;
  msController *thisMscDrive;
};



// Use FILE_READ & FILE_WRITE as defined by FS.h
#if defined(FILE_READ) && !defined(FS_H)
#undef FILE_READ
#endif
#if defined(FILE_WRITE) && !defined(FS_H)
#undef FILE_WRITE
#endif
#include <FS.h>

#define MSC_MAX_FILENAME_LEN 256

class MSCFile : public FileImpl
{
private:
	// Classes derived from File are never meant to be constructed
	// anywhere other than open() in the parent FS class and
	// openNextFile() while traversing a directory.
	// Only the abstract File class which references these derived
	// classes is meant to have a public constructor!
	MSCFile(const FsFile &file) : mscfatfile(file), filename(nullptr) { }
	friend class MSCClass;
public:
	virtual ~MSCFile(void) {
		if (mscfatfile) mscfatfile.close();
		if (filename) free(filename);
	}
#ifdef FILE_WHOAMI
	virtual void whoami() {
		Serial.printf("   MSCFile this=%x, refcount=%u\n",
			(int)this, getRefcount());
	}
#endif
	virtual size_t write(const void *buf, size_t size) {
		return mscfatfile.write(buf, size);
	}
	virtual int peek() {
		return mscfatfile.peek();
	}
	virtual int available() {
		return mscfatfile.available();
	}
	virtual void flush() {
		mscfatfile.flush();
	}
	virtual size_t read(void *buf, size_t nbyte) {
		return mscfatfile.read(buf, nbyte);
	}
	virtual bool truncate(uint64_t size=0) {
		return mscfatfile.truncate(size);
	}
	virtual bool seek(uint64_t pos, int mode = SeekSet) {
		if (mode == SeekSet) return mscfatfile.seekSet(pos);
		if (mode == SeekCur) return mscfatfile.seekCur(pos);
		if (mode == SeekEnd) return mscfatfile.seekEnd(pos);
		return false;
	}
	virtual uint64_t position() {
		return mscfatfile.curPosition();
	}
	virtual uint64_t size() {
		return mscfatfile.size();
	}
	virtual void close() {
		if (filename) {
			free(filename);
			filename = nullptr;
		}
		mscfatfile.close();
	}
	virtual bool isOpen() {
		return mscfatfile.isOpen();
	}
	virtual const char * name() {
		if (!filename) {
			filename = (char *)malloc(MSC_MAX_FILENAME_LEN);
			if (filename) {
				mscfatfile.getName(filename, MSC_MAX_FILENAME_LEN);
			} else {
				static char zeroterm = 0;
				filename = &zeroterm;
			}
		}
		return filename;
	}
	virtual boolean isDirectory(void) {
		return mscfatfile.isDirectory();
	}
	virtual File openNextFile(uint8_t mode=0) {
		FsFile file = mscfatfile.openNextFile();
		if (file) return File(new MSCFile(file));
		return File();
	}
	virtual void rewindDirectory(void) {
		mscfatfile.rewindDirectory();
	}
#ifdef FS_FILE_SUPPORT_DATES
	// These will all return false as only some FS support it.
	virtual bool getAccessDateTime(uint16_t* pdate, uint16_t* ptime) {
		return mscfatfile.getAccessDateTime(pdate, ptime);
	}
	virtual bool getCreateDateTime(uint16_t* pdate, uint16_t* ptime) {
		return mscfatfile.getCreateDateTime(pdate, ptime);
	}
	virtual bool getModifyDateTime(uint16_t* pdate, uint16_t* ptime) {
		return mscfatfile.getModifyDateTime(pdate, ptime);
	}
	virtual bool timestamp(uint8_t flags, uint16_t year, uint8_t month, uint8_t day,
               uint8_t hour, uint8_t minute, uint8_t second) {
		return mscfatfile.timestamp(flags, year, month, day, hour, minute, second);
		return false;
	}
#endif

private:
	FsFile mscfatfile;
	char *filename;
};



class MSCClass : public FS
{
public:
	MSCClass() { }
	bool begin(msController *pDrive, bool setCwv = true, uint8_t part = 1) {
		return mscfs.begin(pDrive, setCwv, part);
	}
	File open(const char *filepath, uint8_t mode = FILE_READ) {
		oflag_t flags = O_READ;
		if (mode == FILE_WRITE) { flags = O_RDWR | O_CREAT | O_AT_END; }
		else if (mode == FILE_WRITE_BEGIN) { flags = O_RDWR | O_CREAT; }
		FsFile file = mscfs.open(filepath, flags);
		if (file) return File(new MSCFile(file));
			return File();
	}
	bool exists(const char *filepath) {
		return mscfs.exists(filepath);
	}
	bool mkdir(const char *filepath) {
		return mscfs.mkdir(filepath);
	}
	bool rename(const char *oldfilepath, const char *newfilepath) {
		return mscfs.rename(oldfilepath, newfilepath);
	}
	bool remove(const char *filepath) {
		return mscfs.remove(filepath);
	}
	bool rmdir(const char *filepath) {
		return mscfs.rmdir(filepath);
	}
	uint64_t usedSize() {	
		return  (uint64_t)(mscfs.clusterCount() - mscfs.freeClusterCount())
		  		* (uint64_t)mscfs.bytesPerCluster();
	}
	uint64_t totalSize() {
		return (uint64_t)mscfs.clusterCount() * (uint64_t)mscfs.bytesPerCluster();
	}
public: // allow access, so users can mix MSC & SdFat APIs
	UsbFs mscfs;
};

extern MSCClass MSC;

// do not expose these defines in Arduino sketches or other libraries
#undef MSC_MAX_FILENAME_LEN

#endif // UsbMscFat_h


