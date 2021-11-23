/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * param.logic.c - Crazy parameter system logic source file.
 */
#include <string.h>
#include <errno.h>

#include "config.h"
#include "crtp.h"
#include "param.h"
#include "param_logic.h"
#include "crc32.h"
#include "console.h"
#include "debug.h"
#include "static_mem.h"
#include "storage.h"

#if 0
#define PARAM_DEBUG(fmt, ...) DEBUG_PRINT("D/param " fmt, ## __VA_ARGS__)
#define PARAM_ERROR(fmt, ...) DEBUG_PRINT("E/param " fmt, ## __VA_ARGS__)
#else
#define PARAM_DEBUG(...)
#define PARAM_ERROR(...)
#endif

static const uint8_t typeLength[] = {
  [PARAM_UINT8]  = 1,
  [PARAM_UINT16] = 2,
  [PARAM_UINT32] = 4,
  [PARAM_INT8]   = 1,
  [PARAM_INT16]  = 2,
  [PARAM_INT32]  = 4,
  [PARAM_FLOAT]  = 4,
};

#define CMD_RESET 0
#define CMD_GET_NEXT 1
#define CMD_GET_CRC 2

#define CMD_GET_ITEM    0 // original version: up to 255 entries
#define CMD_GET_INFO    1 // original version: up to 255 entries
#define CMD_GET_ITEM_V2 2 // version 2: up to 16k entries
#define CMD_GET_INFO_V2 3 // version 2: up to 16k entries


//Private functions
static int variableGetIndex(int id);
static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr);


#ifndef UNIT_TEST_MODE
//These are set by the Linker
extern struct param_s _param_start;
extern struct param_s _param_stop;
#else
//When placed in ram for testing
extern struct param_s *_param_start;
extern struct param_s *_param_stop;
#endif


//Pointer to the parameters list and length of it
static struct param_s * params;
static int paramsLen;
static uint32_t paramsCrc;
static uint16_t paramsCount = 0;
// indicates if read/write operation use V2 (i.e., 16-bit index)
// This is set to true, if a client uses TOC_CH in V2
static bool useV2 = false;

// _sdata is from linker script and points to start of data section
extern int _sdata;
extern int _edata;
// sidata is from linker script and points to start of initialization data flash section
extern int _sidata;
// _stext is from linker script and points to start of flash section
extern int _stext;
extern int _etext;
static const uint64_t dummyZero64 = 0;

static void * paramGetDefault(int index)
{
  uint32_t valueRelative;
  uint32_t address;
  void *ptrDefaultValue;

  address = (uint32_t)(params[index].address);

  // Is variable in data section?
  if (address >= (uint32_t)&_sdata &&
      address <= (uint32_t)&_edata)
  {
    valueRelative =  address - (uint32_t)&_sdata;
    ptrDefaultValue = (void *)((uint32_t)&_sidata + valueRelative);
  }
  // Is variable in flash section?
  else if (address >= (uint32_t)&_stext &&
           address <= (uint32_t)&_etext)
  {
    ptrDefaultValue = (void *)(address);
  }
  // It is zero
  else
  {
    ptrDefaultValue = (void *)&dummyZero64;
  }

  return ptrDefaultValue;
}

void paramLogicInit(void)
{
  int i;
  const char* group = NULL;
  int groupLength = 0;
  uint8_t buf[30];

#ifndef UNIT_TEST_MODE
  params = &_param_start;
  paramsLen = &_param_stop - &_param_start;
#else
  params = _param_start;
  paramsLen = _param_stop - _param_start;
#endif
  // Calculate a hash of the toc by chaining description of each elements
  paramsCrc = 0;
  for (int i=0; i<paramsLen; i++)
  {
    int len = 5;
    memcpy(&buf[0], &paramsCrc, 4);
    buf[4] = params[i].type;
    if (params[i].type & PARAM_GROUP) {
      if (params[i].type & PARAM_START) {
        group = params[i].name;
        groupLength = strlen(group);
      }
    } else {
      // CMD_GET_ITEM_V2 result's size is: 4 + strlen(params[i].name) + groupLength + 2
      if (strlen(params[i].name) + groupLength + 2 > 26) {
        PARAM_ERROR("'%s.%s' too long\n", group, params[i].name);
        ASSERT_FAILED();
      }
    }

    if (params[i].name) {
      memcpy(&buf[5], params[i].name, strlen(params[i].name));
      len += strlen(params[i].name);
    }
    paramsCrc = crc32CalculateBuffer(buf, len);
  }

  for (i=0; i<paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP))
      paramsCount++;
  }
}

void paramTOCProcess(CRTPPacket *p, int command)
{
  int ptr = 0;
  char * group = "";
  uint16_t n=0;
  uint16_t paramId=0;

  switch (command)
  {
  case CMD_GET_INFO: //Get info packet about the param implementation
    DEBUG_PRINT("Client uses old param API!\n");
    ptr = 0;
    group = "";
    p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
    p->size=6;
    p->data[0]=CMD_GET_INFO;
    if (paramsCount < 255) {
      p->data[1] = (uint8_t)paramsCount;
    } else {
      p->data[1] = 255;
    }
    memcpy(&p->data[2], &paramsCrc, 4);
    crtpSendPacketBlock(p);
    break;
  case CMD_GET_ITEM:  //Get param variable
    for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
    {
      if (params[ptr].type & PARAM_GROUP)
      {
        if (params[ptr].type & PARAM_START)
          group = params[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==p->data[1])
          break;
        n++;
      }
    }

    if (ptr<paramsLen)
    {
      p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p->data[0]=CMD_GET_ITEM;
      p->data[1]=n;
      p->data[2]=params[ptr].type;
      p->size=3+2+strlen(group)+strlen(params[ptr].name);
      ASSERT(p->size <= CRTP_MAX_DATA_SIZE); // Too long! The name of the group or the parameter may be too long.
      memcpy(p->data+3, group, strlen(group)+1);
      memcpy(p->data+3+strlen(group)+1, params[ptr].name, strlen(params[ptr].name)+1);
      crtpSendPacketBlock(p);
    } else {
      p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p->data[0]=CMD_GET_ITEM;
      p->size=1;
      crtpSendPacketBlock(p);
    }
    break;
  case CMD_GET_INFO_V2: //Get info packet about the param implementation
    ptr = 0;
    group = "";
    p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
    p->size=7;
    p->data[0]=CMD_GET_INFO_V2;
    memcpy(&p->data[1], &paramsCount, 2);
    memcpy(&p->data[3], &paramsCrc, 4);
    crtpSendPacketBlock(p);
    useV2 = true;
    break;
  case CMD_GET_ITEM_V2:  //Get param variable
    memcpy(&paramId, &p->data[1], 2);
    for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
    {
      if (params[ptr].type & PARAM_GROUP)
      {
        if (params[ptr].type & PARAM_START)
          group = params[ptr].name;
        else
          group = "";
      }
      else                          //Ptr points a variable
      {
        if (n==paramId)
          break;
        n++;
      }
    }

    if (ptr<paramsLen)
    {
      p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p->data[0]=CMD_GET_ITEM_V2;
      memcpy(&p->data[1], &paramId, 2);
      p->data[3] = params[ptr].type;
      p->size = 4 + 2 + strlen(group) + strlen(params[ptr].name);
      ASSERT(p->size <= CRTP_MAX_DATA_SIZE); // Too long! The name of the group or the parameter may be too long.
      memcpy(p->data+4, group, strlen(group)+1);
      memcpy(p->data+4+strlen(group)+1, params[ptr].name, strlen(params[ptr].name)+1);
      crtpSendPacketBlock(p);
    } else {
      p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
      p->data[0]=CMD_GET_ITEM_V2;
      p->size=1;
      crtpSendPacketBlock(p);
    }
    break;
  }
}

void paramWriteProcess(CRTPPacket *p)
{
  if (useV2) {
    uint16_t id;
    memcpy(&id, &p->data[0], 2);

    void* valptr = &p->data[2];
    int index;

    index = variableGetIndex(id);

    if (index < 0) {
      p->data[2] = ENOENT;
      p->size = 3;

      crtpSendPacketBlock(p);
      return;
    }

    if (params[index].type & PARAM_RONLY)
      return;

    switch (params[index].type & PARAM_BYTES_MASK)
    {
    case PARAM_1BYTE:
      *(uint8_t*)params[index].address = *(uint8_t*)valptr;
      break;
      case PARAM_2BYTES:
        *(uint16_t*)params[index].address = *(uint16_t*)valptr;
        break;
    case PARAM_4BYTES:
        *(uint32_t*)params[index].address = *(uint32_t*)valptr;
        break;
    case PARAM_8BYTES:
        *(uint64_t*)params[index].address = *(uint64_t*)valptr;
        break;
    }

    crtpSendPacketBlock(p);

    if (params[index].callback) {
      params[index].callback();
    }

  } else {
    int id = p->data[0];
    void* valptr = &p->data[1];
    int index;

    index = variableGetIndex(id);

    if (index<0) {
      p->data[0] = -1;
      p->data[1] = id;
      p->data[2] = ENOENT;
      p->size = 3;

      crtpSendPacketBlock(p);
      return;
    }

  	if (params[index].type & PARAM_RONLY)
  		return;

    switch (params[index].type & PARAM_BYTES_MASK)
    {
   	case PARAM_1BYTE:
   		*(uint8_t*)params[index].address = *(uint8_t*)valptr;
   		break;
      case PARAM_2BYTES:
    	  *(uint16_t*)params[index].address = *(uint16_t*)valptr;
        break;
   	case PARAM_4BYTES:
        *(uint32_t*)params[index].address = *(uint32_t*)valptr;
        break;
   	case PARAM_8BYTES:
        *(uint64_t*)params[index].address = *(uint64_t*)valptr;
        break;
    }

    crtpSendPacketBlock(p);
  }
}

static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr) {
  int index;
  char *pgroup = "";

  for (index = 0; index < paramsLen; index++) //Ptr points a group
  {
    if (params[index].type & PARAM_GROUP)
    {
      if (params[index].type & PARAM_START)
        pgroup = params[index].name;
      else
        pgroup = "";
    }
    else                          //Ptr points a variable
    {
      if (!strcmp(params[index].name, name) && !strcmp(pgroup, group))
        break;
    }
  }

  if (index >= paramsLen) {
    return ENOENT;
  }

  if (type != (params[index].type & (~(PARAM_CORE | PARAM_RONLY)))) {
    return EINVAL;
  }

  if (params[index].type & PARAM_RONLY) {
    return EACCES;
  }

  switch (params[index].type & PARAM_BYTES_MASK)
  {
 	case PARAM_1BYTE:
 		*(uint8_t*)params[index].address = *(uint8_t*)valptr;
 		break;
    case PARAM_2BYTES:
  	  *(uint16_t*)params[index].address = *(uint16_t*)valptr;
      break;
 	case PARAM_4BYTES:
      *(uint32_t*)params[index].address = *(uint32_t*)valptr;
      break;
 	case PARAM_8BYTES:
      *(uint64_t*)params[index].address = *(uint64_t*)valptr;
      break;
  }

  return 0;
}

void paramReadProcess(CRTPPacket *p)
{
  if (useV2) {
    uint16_t id;
    memcpy(&id, &p->data[0], 2);
    int index = variableGetIndex(id);

    if (index<0) {
      p->data[2] = ENOENT;
      p->size = 3;

      crtpSendPacketBlock(p);
      return;
    }

    p->data[2] = 0;
    switch (params[index].type & PARAM_BYTES_MASK)
    {
      case PARAM_1BYTE:
        memcpy(&p->data[3], params[index].address, sizeof(uint8_t));
        p->size = 3+sizeof(uint8_t);
        break;
      break;
      case PARAM_2BYTES:
        memcpy(&p->data[3], params[index].address, sizeof(uint16_t));
        p->size = 3+sizeof(uint16_t);
        break;
      case PARAM_4BYTES:
        memcpy(&p->data[3], params[index].address, sizeof(uint32_t));
        p->size = 3+sizeof(uint32_t);
        break;
      case PARAM_8BYTES:
        memcpy(&p->data[3], params[index].address, sizeof(uint64_t));
        p->size = 3+sizeof(uint64_t);
        break;
    }
  } else {
    uint8_t id = p->data[0];
    int index = variableGetIndex(id);

    if (index < 0) {
      p->data[0] = -1;
      p->data[1] = id;
      p->data[2] = ENOENT;
      p->size = 3;

      crtpSendPacketBlock(p);
      return;
    }

    switch (params[index].type & PARAM_BYTES_MASK)
    {
      case PARAM_1BYTE:
     		memcpy(&p->data[1], params[index].address, sizeof(uint8_t));
     		p->size = 1+sizeof(uint8_t);
     		break;
   		break;
      case PARAM_2BYTES:
     		memcpy(&p->data[1], params[index].address, sizeof(uint16_t));
     		p->size = 1+sizeof(uint16_t);
     		break;
      case PARAM_4BYTES:
        memcpy(&p->data[1], params[index].address, sizeof(uint32_t));
     		p->size = 1+sizeof(uint32_t);
     		break;
   	  case PARAM_8BYTES:
        memcpy(&p->data[1], params[index].address, sizeof(uint64_t));
     		p->size = 1+sizeof(uint64_t);
     		break;
    }
  }

  crtpSendPacketBlock(p);
}

static int variableGetIndex(int id)
{
  int i;
  int n = 0;

  for (i = 0; i < paramsLen; i++)
  {
    if(!(params[i].type & PARAM_GROUP))
    {
      if(n == id) {
        break;
      }
      n++;
    }
  }

  if (i >= paramsLen)
    return -1;

  return i;
}

/* Public API to access param TOC from within the copter */
static paramVarId_t invalidVarId = {0xffffu, 0xffffu};

paramVarId_t paramGetVarId(const char* group, const char* name)
{
  uint16_t index;
  uint16_t id = 0;
  paramVarId_t varId = invalidVarId;
  char * currgroup = "";

  for(index = 0; index < paramsLen; index++)
  {
    if (params[index].type & PARAM_GROUP) {
      if (params[index].type & PARAM_START) {
        currgroup = params[index].name;
      }
    } else {
      id += 1;
    }
    
    if ((!strcmp(group, currgroup)) && (!strcmp(name, params[index].name))) {
      varId.index = index;
      varId.id = id - 1;
      return varId;
    }
  }

  return invalidVarId;
}

int paramGetType(paramVarId_t varid)
{
  return params[varid.index].type;
}

void paramGetGroupAndName(paramVarId_t varid, char** group, char** name)
{
  char * currgroup = "";
  *group = 0;
  *name = 0;

  for(int index = 0; index < paramsLen; index++) {
    if (params[index].type & PARAM_GROUP) {
      if (params[index].type & PARAM_START) {
        currgroup = params[index].name;
      }
    }

    if (index == varid.index) {
      *group = currgroup;
      *name = params[index].name;
      break;
    }
  }
}

uint8_t paramVarSize(int type)
{
  return typeLength[type];
}

int paramGetInt(paramVarId_t varid)
{
  int valuei = 0;

  ASSERT(PARAM_VARID_IS_VALID(varid));

  switch(params[varid.index].type & (~(PARAM_CORE | PARAM_RONLY)))
  {
    case PARAM_UINT8:
      valuei = *(uint8_t *)params[varid.index].address;
      break;
    case PARAM_INT8:
      valuei = *(int8_t *)params[varid.index].address;
      break;
    case PARAM_UINT16:
      valuei = *(uint16_t *)params[varid.index].address;
      break;
    case PARAM_INT16:
      valuei = *(int16_t *)params[varid.index].address;
      break;
    case PARAM_UINT32:
      valuei = *(uint32_t *)params[varid.index].address;
      break;
    case PARAM_INT32:
      valuei = *(int32_t *)params[varid.index].address;
      break;
    case PARAM_FLOAT:
      valuei = *(float *)params[varid.index].address;
      break;
  }

  return valuei;
}

float paramGetFloat(paramVarId_t varid)
{
  ASSERT(PARAM_VARID_IS_VALID(varid));

  if ((params[varid.index].type & (~(PARAM_CORE | PARAM_RONLY))) == PARAM_FLOAT)
    return *(float *)params[varid.index].address;

  return (float)paramGetInt(varid);
}

unsigned int paramGetUint(paramVarId_t varid)
{
  return (unsigned int)paramGetInt(varid);
}

void paramSetInt(paramVarId_t varid, int valuei)
{
  ASSERT(PARAM_VARID_IS_VALID(varid));

  static CRTPPacket pk;
  pk.header=CRTP_HEADER(CRTP_PORT_PARAM, MISC_CH);
  pk.data[0] = MISC_VALUE_UPDATED;
  pk.data[1] = varid.id & 0xffu;
  pk.data[2] = (varid.id >> 8) & 0xffu;
  pk.size = 3;

  switch(params[varid.index].type & (~PARAM_CORE))
  {
    case PARAM_UINT8:
      *(uint8_t *)params[varid.index].address = (uint8_t) valuei;
      memcpy(&pk.data[2], &valuei, 1);
      pk.size += 1;
      break;
    case PARAM_INT8:
      *(int8_t *)params[varid.index].address = (int8_t) valuei;
      memcpy(&pk.data[2], &valuei, 1);
      pk.size += 1;
      break;
    case PARAM_UINT16:
      *(uint16_t *)params[varid.index].address = (uint16_t) valuei;
      memcpy(&pk.data[2], &valuei, 2);
      pk.size += 2;
      break;
    case PARAM_INT16:
      *(int16_t *)params[varid.index].address = (int16_t) valuei;
      memcpy(&pk.data[2], &valuei, 2);
      pk.size += 2;
      break;
    case PARAM_UINT32:
      *(uint32_t *)params[varid.index].address = (uint32_t) valuei;
      memcpy(&pk.data[2], &valuei, 4);
      pk.size += 4;
      break;
    case PARAM_INT32:
      *(int32_t *)params[varid.index].address = (int32_t) valuei;
      memcpy(&pk.data[2], &valuei, 4);
      pk.size += 4;
      break;
    case PARAM_FLOAT:
    // Todo: are floats handy to have here?
      *(float *)params[varid.index].address = (float) valuei;
      memcpy(&pk.data[2], &valuei, 4);
      pk.size += 4;

      break;
  }

#ifndef SILENT_PARAM_UPDATES
  crtpSendPacketBlock(&pk);
#endif
}

void paramSetFloat(paramVarId_t varid, float valuef)
{
  ASSERT(PARAM_VARID_IS_VALID(varid));

  static CRTPPacket pk;
  pk.header=CRTP_HEADER(CRTP_PORT_PARAM, MISC_CH);
  pk.data[0] = MISC_VALUE_UPDATED;
  pk.data[1] = varid.id & 0xffu;
  pk.data[2] = (varid.id >> 8) & 0xffu;
  pk.size=3;

  if ((params[varid.index].type & (~PARAM_CORE)) == PARAM_FLOAT) {
      *(float *)params[varid.index].address = valuef;

      memcpy(&pk.data[2], &valuef, 4);
      pk.size += 4;
  }

#ifndef SILENT_PARAM_UPDATES
  crtpSendPacketBlock(&pk);
#endif
}

void paramSetByName(CRTPPacket *p)
{
  int i, nzero = 0;
  char *group;
  char *name;
  uint8_t type;
  void * valPtr;
  int error;

  // If the packet contains at least 2 zeros in the first 28 bytes
  // The packet decoding algorithm will not crash
  for (i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
    if (p->data[i] == '\0') nzero++;
  }

  if (nzero < 2) return;

  group = (char*)&p->data[1];
  name = (char*)&p->data[1 + strlen(group) + 1];
  type = p->data[1 + strlen(group) + 1 + strlen(name) + 1];
  valPtr = &p->data[1 + strlen(group) + 1 + strlen(name) + 2];

  error = paramWriteByNameProcess(group, name, type, valPtr);

  p->data[1 + strlen(group) + 1 + strlen(name) + 1] = error;
  p->size = 1 + strlen(group) + 1 + strlen(name) + 1 + 1;
  crtpSendPacketBlock(p);

}

#define KEY_LEN 30  // FIXME

void paramGetExtendedType(CRTPPacket *p)
{
  int index;
  uint16_t id;

  memcpy(&id, &p->data[1], 2);
  index = variableGetIndex(id);

  if (index < 0 || !(params[index].type & PARAM_EXTENDED)) {
    p->data[3] = ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
    return;
  }

  p->data[3] = params[index].extended_type;
  p->size = 4;

  crtpSendPacketBlock(p);
}


void paramPersistentStore(CRTPPacket *p)
{
  int index;
  char *group;
  char *name;
  uint16_t id;
  bool result = true;
  paramVarId_t paramId;

  memcpy(&id, &p->data[1], 2);
  index = variableGetIndex(id);

  if (index < 0) {
    p->data[3] = ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
    return;
  }

  paramId.index = (uint16_t)index;
  paramGetGroupAndName(paramId, &group, &name);

  // Assemble key string, e.g. "prm/pid_rate.kp"
  char key[KEY_LEN] = {0};
  strcpy(key, PERSISTENT_PREFIX_STRING);
  strcat(key, group);
  strcat(key, ".");
  strcat(key, name);

  switch (params[index].type & PARAM_BYTES_MASK) {
    case PARAM_1BYTE:
      result = storageStore(key, params[index].address, 1);
      break;
    case PARAM_2BYTES:
      result = storageStore(key, params[index].address, 2);
      break;
    case PARAM_4BYTES:
      result = storageStore(key, params[index].address, 4);
      break;
    case PARAM_8BYTES:
      result = storageStore(key, params[index].address, 8);
      break;
  }

  p->data[3] = result ? 0: ENOENT;
  p->size = 4;
  crtpSendPacketBlock(p);
}

void paramPersistentGetState(CRTPPacket *p)
{
  int index;
  char *group;
  char *name;
  size_t varSize = 0;
  uint16_t id;
  paramVarId_t paramId;
  uint8_t value[8];

  memcpy(&id, &p->data[1], 2);
  index = variableGetIndex(id);

  if (index < 0) {
    p->data[3] = ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
    return;
  }

  paramId.index = (uint16_t)index;
  paramGetGroupAndName(paramId, &group, &name);

  // Assemble key string, e.g. "prm/pid_rate.kp"
  char key[KEY_LEN] = {0};
  strcpy(key, PERSISTENT_PREFIX_STRING);
  strcat(key, group);
  strcat(key, ".");
  strcat(key, name);

  if (storageFetch(key, &value, 8)) {
    p->data[3] = PARAM_PERSISTENT_STORED; // Value is stored
    p->size = 4;
    switch (params[index].type & PARAM_BYTES_MASK) {
      case PARAM_1BYTE:
        memcpy(&p->data[4], params[index].getter(), sizeof(uint8_t));
        memcpy(&p->data[5], &value, sizeof(uint8_t));
        varSize = 2 * sizeof(uint8_t);
        break;
      break;
      case PARAM_2BYTES:
        memcpy(&p->data[4], params[index].getter(), sizeof(uint16_t));
        memcpy(&p->data[6], &value, sizeof(uint16_t));
        varSize =  2 * sizeof(uint16_t);
        break;
      case PARAM_4BYTES:
        memcpy(&p->data[4], params[index].getter(), sizeof(uint32_t));
        memcpy(&p->data[8], &value, sizeof(uint32_t));
        varSize = 2 * sizeof(uint32_t);
        break;
      case PARAM_8BYTES:
        memcpy(&p->data[4], params[index].getter(), sizeof(uint64_t));
        memcpy(&p->data[12], &value, sizeof(uint64_t));
        varSize = 2 * sizeof(uint64_t);
        break;
    }
  } else {
    p->data[3] = PARAM_PERSISTENT_NOT_STORED; // Value is not stored
    p->size = 4;
    if (params[index].getter)
    {
      switch (params[index].type & PARAM_BYTES_MASK) {
        case PARAM_1BYTE:
          memcpy(&p->data[4], params[index].getter(), sizeof(uint8_t));
          varSize = sizeof(uint8_t);
          break;
        break;
        case PARAM_2BYTES:
          memcpy(&p->data[4], params[index].getter(), sizeof(uint16_t));
          varSize = sizeof(uint16_t);
          break;
        case PARAM_4BYTES:
          memcpy(&p->data[4], params[index].getter(), sizeof(uint32_t));
          varSize = sizeof(uint32_t);
          break;
        case PARAM_8BYTES:
          memcpy(&p->data[4], params[index].getter(), sizeof(uint64_t));
          varSize = sizeof(uint64_t);
          break;
      }
    } else {
      switch (params[index].type & PARAM_BYTES_MASK) {
        case PARAM_1BYTE:
          memcpy(&p->data[4], paramGetDefault(index), sizeof(uint8_t));
          varSize = sizeof(uint8_t);
          break;
        break;
        case PARAM_2BYTES:
          memcpy(&p->data[4], paramGetDefault(index), sizeof(uint16_t));
          varSize = sizeof(uint16_t);
          break;
        case PARAM_4BYTES:
          memcpy(&p->data[4], paramGetDefault(index), sizeof(uint32_t));
          varSize = sizeof(uint32_t);
          break;
        case PARAM_8BYTES:
          memcpy(&p->data[4], paramGetDefault(index), sizeof(uint64_t));
          varSize = sizeof(uint64_t);
          break;
      }
    }
  }

  p->size = p->size + (uint8_t)(varSize & 0xFF);
  crtpSendPacketBlock(p);
}

void paramPersistentClear(CRTPPacket *p)
{
  int index;
  char *group;
  char *name;
  uint16_t id;
  bool result = true;
  paramVarId_t paramId;

  memcpy(&id, &p->data[1], 2);
  index = variableGetIndex(id);

  if (index < 0) {
    p->data[3] = ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
    return;
  }

  paramId.index = (uint16_t)index;
  paramGetGroupAndName(paramId, &group, &name);

  // Assemble key string, e.g. "prm/pid_rate.kp"
  char key[KEY_LEN] = {0};
  strcpy(key, PERSISTENT_PREFIX_STRING);
  strcat(key, group);
  strcat(key, ".");
  strcat(key, name);

  result = storageDelete(key);

  p->data[3] = result ? 0: ENOENT;
  p->size = 4;
  crtpSendPacketBlock(p);
}

