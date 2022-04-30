/**
 * Copyright (c) 2011-2020 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
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
#ifndef PFsExFatFormatter_h
#define PFsExFatFormatter_h
#include "mscFS.h"
#include <SdFat.h>

//#include "ExFatConfig.h"
//#include "../common/SysCall.h"
//#include "../common/BlockDevice.h"
/**
 * \class ExFatFormatter
 * \brief Format an exFAT volume.
 */
class PFsExFatFormatter {
 public:
  /**
   * Format an exFAT volume.
   *
   * \param[in] dev Block device for volume.
   * \param[in] secBuf buffer for writing to volume.
   * \param[in] pr Print device for progress output.
   *
   * \return true for success or false for failure.
   */
  bool format(PFsVolume &partVol, uint8_t* secBuf, print_t* pr);
  bool createExFatPartition(FsBlockDevice* dev, uint32_t startSector, uint32_t sectorCount, uint8_t* secBuf, print_t* pr);
  uint8_t addExFatPartitionToMbr();
  void dump_hexbytes(const void *ptr, int len);

 private:
 
  bool writeMbr();
  bool syncUpcase();
  bool writeUpcase(uint32_t sector);
  bool writeUpcaseByte(uint8_t b);
  bool writeUpcaseUnicode(uint16_t unicode);
  
  uint32_t m_upcaseSector;
  uint32_t m_upcaseChecksum;
  uint32_t m_upcaseSize;
  FsBlockDevice* m_dev;
  print_t* m_pr;
  uint8_t* m_secBuf;
  uint32_t bitmapSize;
  uint32_t checksum = 0;
  uint32_t clusterCount;
  uint32_t clusterHeapOffset;
  uint32_t fatLength;
  uint32_t fatOffset;
  uint32_t m;
  uint32_t ns;
  uint32_t partitionOffset;
  uint32_t sector;
  uint32_t sectorsPerCluster;
  uint32_t volumeLength;
  uint32_t sectorCount;
  uint8_t sectorsPerClusterShift;
  uint32_t m_relativeSectors;
  uint32_t m_part_relativeSectors;
  uint32_t m_bytesPerCluster;
  uint8_t m_part;
  uint32_t m_sectorCount;
  uint32_t m_capacityMB;
  char volName[32];
  
};
#endif  // ExFatFormatter_h
