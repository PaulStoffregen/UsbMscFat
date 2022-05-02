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

#ifndef UsbMscFat_h
#define UsbMscFat_h


const char *decodeSenseKey(uint8_t senseKey);
const char *decodeAscAscq(uint8_t asc, uint8_t ascq);

void printMscAscError(print_t* pr, msController *pDrive);

inline uint32_t USBmscCapacity(msController *pDrv) {
	return (pDrv->msDriveInfo.capacity.Blocks); 
}



/**
 * \class USBMSCDevice  TODO: to become part of msController class
 * \brief Raw USB Drive accesss.
 */
class USBMSCDevice : public FsBlockDeviceInterface {
 public:
  constexpr USBMSCDevice() {}
  // Initialize the USB MSC device.
  bool begin(msController *pDrive);
  // return the number of 512 byte sectors for the whole drive
  uint32_t sectorCount() { return thisDrive->msDriveInfo.capacity.Blocks; }
  // return code for the last error.  (where is list of errors?)
  uint8_t errorCode() const { return m_errorCode; }
  // return error data for last error.
  uint32_t errorData() const { return 0; }
  // return error line for last error. Tmp function for debug.
  uint32_t errorLine() const { return m_errorLine; }
  // Check for busy
  bool isBusy() { return !m_initDone && !thisDrive->mscTransferComplete; }
  // Check for busy with MSC read operation
  bool isBusyRead() { return thisDrive->mscTransferComplete; }
  // Check for busy with MSC read operation
  bool isBusyWrite() { return thisDrive->mscTransferComplete; }
  // Read a USB drive's information. This contains the drive's identification
  // information such as Manufacturer ID, Product name, Product serial
  // number and Manufacturing date pluse more.
  bool readUSBDriveInfo(msDriveInfo_t * driveInfo) {
    memcpy(driveInfo, &thisDrive->msDriveInfo, sizeof(msDriveInfo_t));
    return true;
  }
  // Read a 512 byte sector from an USB MSC drive.
  bool readSector(uint32_t sector, uint8_t* dst);
  // Read multiple 512 byte sectors from an USB MSC drive.
  bool readSectors(uint32_t sector, uint8_t* dst, size_t numsectors);
  // return USB MSC drive status.
  uint32_t status() { return m_errorCode; }
  // return success if sync successful. Not for user apps.
  bool syncDevice() { return true; }
  // Writes a 512 byte sector to an USB MSC drive.
  bool writeSector(uint32_t sector, const uint8_t* src);
  // Write multiple 512 byte sectors to an USB MSC drive.
  bool writeSectors(uint32_t sector, const uint8_t* src, size_t ns);
  // Read multiple 512 byte sectors from an USB MSC drive, using
  // a callback per sector  TODO: this is not used by FsBlockDeviceInterface
  bool readSectorsWithCB(uint32_t sector, size_t ns,
    void (*callback)(uint32_t, uint8_t *), uint32_t token);

private:
  msController *thisDrive = nullptr;
  bool m_initDone = false;
  uint8_t m_errorCode = MS_NO_MEDIA_ERR;
  uint32_t m_errorLine = 0;
  friend class UsbFs;
};




/** MSCFat version */
#define MSC_FAT_VERSION "1.0.0"

//==============================================================================

class UsbFs : public FsVolume {
private:
  // UsbFs are only created as part of MSCClass.  To call SdFat functions,
  // you can access UsbFs within MSCClass as member "mscfs".
  UsbFs() {}
  friend class MSCClass;
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
    if (device.errorCode() != 0) return false;
    // Serial.println("    After usbDriveBegin");
    return FsVolume::begin(&device, setCwv, part);
  }
  //---------------------------------------------------------------------------
  // TODO: programs using these should be offered a better API for their needs....
  FsVolume * vol() { return this; } // is this redundant?
  bool volumeBegin() { return FsVolume::begin(&device); } // is this redundant?
  FsBlockDeviceInterface * usbDrive() { return &device; } // used by VolumeName.ino
  //---------------------------------------------------------------------------
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
  // Error Message Printing
  void errorHalt(print_t* pr);
  void errorHalt(print_t* pr, const char* msg);
  void errorHalt(print_t* pr, const __FlashStringHelper* msg);
  void initErrorHalt(print_t* pr);
  void initErrorHalt(print_t* pr, const char* msg);
  void initErrorHalt(Print* pr, const __FlashStringHelper* msg);
  void initErrorPrint(Print* pr);
  void printFatType(print_t* pr);
  void errorPrint(print_t* pr);
  void errorPrint(print_t* pr, char const* msg);
  void errorPrint(Print* pr, const __FlashStringHelper* msg);
  void printMscError(print_t* pr);
  uint8_t mscErrorCode() { return device.errorCode(); }
  uint8_t mscErrorData() { return device.errorData(); }
#if ENABLE_ARDUINO_SERIAL
  void initErrorPrint() { initErrorPrint(&Serial); }
  void errorHalt(const __FlashStringHelper* msg) { errorHalt(&Serial, msg); }
  void errorHalt() { errorHalt(&Serial); }
  void errorHalt(const char* msg) { errorHalt(&Serial, msg); }
  void initErrorHalt() { initErrorHalt(&Serial); }
  void errorPrint(const char* msg) { errorPrint(&Serial, msg); }
  void errorPrint(const __FlashStringHelper* msg) { errorPrint(&Serial, msg); }
  void initErrorHalt(const char* msg) {initErrorHalt(&Serial, msg);}
  void initErrorHalt(const __FlashStringHelper* msg) { initErrorHalt(&Serial, msg); }
#endif  // ENABLE_ARDUINO_SERIAL
  //----------------------------------------------------------------------------
private:
  USBMSCDevice device; // to become a pointer when API moves away from begin()
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
public: // allow access, so users can access SdFat APIs
	UsbFs mscfs;
};

extern MSCClass MSC;

// do not expose these defines in Arduino sketches or other libraries
#undef MSC_MAX_FILENAME_LEN

#endif // UsbMscFat_h


