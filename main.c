/*
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <unistd.h>
#include <assert.h>
#include <sys/stat.h>

#include <libusb-1.0/libusb.h>

#include "ff.h"


#if defined(_MSC_VER)
#define snprintf _snprintf
#define putenv _putenv
#endif

// Future versions of libusb will use usb_interface instead of interface
// in libusb_config_descriptor => catter for that
#define usb_interface interface

// Global variables
static bool binary_dump = false;
static bool extra_info = false;
static bool force_device_request = false;	// For WCID descriptor queries
static const char* binary_name = NULL;

static inline void msleep(int msecs)
{
#if defined(_WIN32)
	Sleep(msecs);
#else
	const struct timespec ts = { msecs / 1000, (msecs % 1000) * 1000000L };
	nanosleep(&ts, NULL);
#endif
}

static void perr(char const *format, ...)
{
	va_list args;

	va_start (args, format);
	vfprintf(stderr, format, args);
	va_end(args);
}

#define ERR_EXIT(errcode) do { perr("   %s\n", libusb_strerror((enum libusb_error)errcode)); return -1; } while (0)
#define CALL_CHECK(fcall) do { int _r=fcall; if (_r < 0) ERR_EXIT(_r); } while (0)
#define CALL_CHECK_CLOSE(fcall, hdl) do { int _r=fcall; if (_r < 0) { libusb_close(hdl); ERR_EXIT(_r); } } while (0)
#define B(x) (((x)!=0)?1:0)
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])

#define RETRY_MAX                     5
#define REQUEST_SENSE_LENGTH          0x12
#define INQUIRY_LENGTH                0x24
#define READ_CAPACITY_LENGTH          0x08

// HID Class-Specific Requests values. See section 7.2 of the HID specifications
#define HID_GET_REPORT                0x01
#define HID_GET_IDLE                  0x02
#define HID_GET_PROTOCOL              0x03
#define HID_SET_REPORT                0x09
#define HID_SET_IDLE                  0x0A
#define HID_SET_PROTOCOL              0x0B
#define HID_REPORT_TYPE_INPUT         0x01
#define HID_REPORT_TYPE_OUTPUT        0x02
#define HID_REPORT_TYPE_FEATURE       0x03

// Mass Storage Requests values. See section 3 of the Bulk-Only Mass Storage Class specifications
#define BOMS_RESET                    0xFF
#define BOMS_GET_MAX_LUN              0xFE

// Microsoft OS Descriptor
#define MS_OS_DESC_STRING_INDEX		0xEE
#define MS_OS_DESC_STRING_LENGTH	0x12
#define MS_OS_DESC_VENDOR_CODE_OFFSET	0x10
static const uint8_t ms_os_desc_string[] = {
	MS_OS_DESC_STRING_LENGTH,
	LIBUSB_DT_STRING,
	'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
};

// Section 5.1: Command Block Wrapper (CBW)
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

// Section 5.2: Command Status Wrapper (CSW)
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static const uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

static enum test_type {
	USE_GENERIC,
	USE_PS3,
	USE_XBOX,
	USE_SCSI,
	USE_HID,
} test_mode;
static uint16_t VID, PID;

static libusb_context *CONTEXT;
static int FD;

static void display_buffer_hex(unsigned char *buffer, unsigned size)
{
	unsigned i, j, k;

	for (i=0; i<size; i+=16) {
		printf("\n  %08x  ", i);
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				printf("%02x", buffer[i+j]);
			} else {
				printf("  ");
			}
			printf(" ");
		}
		printf(" ");
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				if ((buffer[i+j] < 32) || (buffer[i+j] > 126)) {
					printf(".");
				} else {
					printf("%c", buffer[i+j]);
				}
			}
		}
	}
	printf("\n" );
}

static char* uuid_to_string(const uint8_t* uuid)
{
	static char uuid_string[40];
	if (uuid == NULL) return NULL;
	snprintf(uuid_string, sizeof(uuid_string),
		"{%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x}",
		uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
		uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
	return uuid_string;
}

////
libusb_device_handle *devh;
static int ep_in_addr = 0x83;
static int ep_out_addr = 0x02;

// #define FF_STORAGE_SIZE (1024*1024)
// unsigned char ff_storage[FF_STORAGE_SIZE];
FATFS fs;

#ifndef MODE
#define MODE (2)
#endif

#define MODE_1_MONITOR (1)
#define MODE_2_TOUCH (2)

#define ACM_CTRL_DTR 0x001
#define ACM_CTRL_RTS 0x002

#if 1
// android
#define errno_t int
int fopen_s(FILE **fp, const char *filename, const char *mode) {
    *fp = fopen(filename, mode);
    if (*fp == 0) {
        return -1;
    }
    return 0;
}
#endif

int write_chars(const char *c) {
    int actual_length;
    int size = (int)(strlen(c));
    if (libusb_bulk_transfer(devh, ep_out_addr, (unsigned char *)c, size, &actual_length, 0) <0) {
        fprintf(stderr, "Error while sending char\n");
    }
    return actual_length;
}

int read_chars(unsigned char *data, int size) {
    int actual_length;
    int rc = libusb_bulk_transfer(devh, ep_in_addr, data, size, &actual_length, 1000);
    if (rc == LIBUSB_ERROR_TIMEOUT) {
        //printf("timeout (%d)\n", actual_length);
        return -1;
    } else if (rc < 0) {
        fprintf(stderr, "Error while waiting for char\n");
        return -1;
    }

    return actual_length;
}

void dump(unsigned char *data, int size) {
    int i, s;
    s = 0;
    for (i = 0; i < size; i++) {
        if (i % 16 == 0) {
            printf("  ");
        }
        printf("%02X ", data[i]);
        if (i % 16 == 15) {
            printf(" ");
            for (int j = s; j <= i; j++) {
                if (0x20 <= data[j] && data[j] <= 0x7E) {
                    printf("%c", data[j]);
                } else {
                    printf(".");
                }
            }
            printf("\n");
            s = i + 1;
        }
    }
    int k;
    k = 16 - ((i - 1) % 16) - 1;
    for (int j = 0; j < k; j++) {
        printf("   ");
    }
    printf(" ");


    for (int j = s; j < i; j++) {
        if (0x20 <= data[j] && data[j] <= 0x7E) {
            printf("%c", data[j]);
        } else {
            printf(".");
        }
    }
    printf("\n");
    s = i;
}

#if 0
#define MSC_ENDPOINT_OUT (0x02)
#define MSC_ENDPOINT_IN  (0x81)
int msc_out(char *msg, unsigned char *data, int size, int verbose) {
    int actual_length;
    int rc = libusb_bulk_transfer(devh, MSC_ENDPOINT_OUT, data, size, &actual_length, 1000);
    if (rc < 0) {
        fprintf(stderr, "Error during bulk transfer: %s\n", libusb_error_name(rc));
    }
    if (verbose > 0) {
        printf("%s : %d\n", msg, actual_length);
    }
    if (verbose >= 10) {
        dump(data, actual_length);
    }
    return rc;
}

int msc_in(char *msg, unsigned char *data, int *size, int verbose) {
    int sz = *size;
    int rc = libusb_bulk_transfer(devh, MSC_ENDPOINT_IN, data, sz, size, 1000);
    if (rc < 0) {
        fprintf(stderr, "Error during bulk transfer: %s\n", libusb_error_name(rc));
    }
    if (verbose > 0) {
        printf("%s : %d\n", msg, *size);
    }
    if (verbose >= 10) {
        dump(data, *size);
    }
    return rc;
}

int msc_0x12_inquiry(unsigned char lun, int verbose) {
    int rc;
    // msc : Inquiry LUN: 0x00
    unsigned char data[64] = {
        //0x55, 0x53, 0x42, 0x43, // Signature
        //0xA0, 0x59, 0x9b, 0x18, // Tag
        //0xFF, 0x00, 0x00, 0x00, // DataTransferLength
        //0x80, 0x00 | lun, 0x06, // Flag, LUN, CBWCBLength
        //0x12, 0x01, 0x80, 0x00, 0xFF, 0x00, 0x00, 0x00,
        //0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        0x55, 0x53, 0x42, 0x43,
        0x10, 0x40, 0x45, 0xbf,
        0x24, 0x00, 0x00, 0x00,
        0x80, 0x00 | lun, 0x06,
        0x12, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    int size;

    rc = msc_out("SCSI: Inquiry LUN: 0x00", data, 31, verbose);
    if (rc < 0) {
        return rc;
    }

    size = 64;
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        return rc;
    }

    size = 64;
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        return rc;
    }

    return rc;
}

int msc_0x25_readcapacity(unsigned char lun, int verbose) {
    int rc;
    // msc : Inquiry LUN: 0x00
    unsigned char data[64] = {
        0x55, 0x53, 0x42, 0x43, // Signature
        0xA0, 0x59, 0x9b, 0x18, // Tag
        0x08, 0x00, 0x00, 0x00, // DataTransferLength
        0x80, 0x00 | lun, 0x0A, // Flag, LUN, CBWCBLength
        0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    int size;

    rc = msc_out("SCSI: Read Capacity LUN: 0x00", data, 31, verbose);
    if (rc < 0) {
        return rc;
    }

    size = 64;
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        return rc;
    }

    if (verbose >= 10) {
        unsigned long llba = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + (data[3]);
        unsigned long blib = (data[4] << 24) + (data[5] << 16) + (data[6] << 8) + (data[7]);
        printf("  Toal: %ld, Block Length in Bytes: %ld\n", llba * blib, blib);
    }

    size = 64;
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        return rc;
    }

    return rc;
}

int msc_0x28_read10(unsigned char lun, int verbose, unsigned long lba, unsigned short count, unsigned char *buf512) {
    int rc;

    unsigned char data[4096] = {
	0x55, 0x53, 0x42, 0x43,
	0x10, 0x50, 0x5a, 0xc1,
	0x00, 0x02, 0x00, 0x00,
	0x80, 0x00 | lun, 0x0a,
	0x28, 0x00,
        // 0x55, 0x53, 0x42, 0x43, // Signature
        // 0xA0, 0x59, 0x9b, 0x18, // Tag
        // 0x00, 0x10, 0x00, 0x00, // DataTransferLength
        // 0x80, 0x00 | lun, 0x0A, // Flag, LUN, CBWCBLength
        // 0x28, 0x00,
        (unsigned char)(lba >> 24),
        (unsigned char)(lba >> 16),
        (unsigned char)(lba >> 8),
        (unsigned char)(lba >> 0),
        0x00,
        (unsigned char)(count >> 8),
        (unsigned char)(count),
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    int size;

    rc = msc_out("SCSI: Read10 LUN: 0x00", data, 31, verbose);
    if (rc < 0) {
        return rc;
    }

    size = 512;
    rc = msc_in("SCSI: Data In LUN", buf512, &size, verbose);
    if (rc < 0) {
        printf("1\n");
        return rc;
    }

    size = sizeof(data);
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        printf("1\n");
        return rc;
    }

    return rc;
}

int msc_0x2A_write10(unsigned char lun, int verbose, unsigned long lba, unsigned short count, unsigned char *buf) {
    int rc;

    unsigned char data[4096] = {
        0x55, 0x53, 0x42, 0x43,
	0x10, 0x50, 0x5a, 0xc1,
	0x00, 0x10, 0x00, 0x00,
	0x00, 0x00 | lun, 0x0a,
	0x2a, 0x00,
        // 0x55, 0x53, 0x42, 0x43, // Signature
        // 0xA0, 0x59, 0x9b, 0x18, // Tag
        // 0x00, 0x10, 0x00, 0x00, // DataTransferLength
        // 0x80, 0x00 | lun, 0x0A, // Flag, LUN, CBWCBLength
        // 0x2A, 0x00,
        (unsigned char)(lba >> 24),
        (unsigned char)(lba >> 16),
        (unsigned char)(lba >> 8),
        (unsigned char)(lba >> 0),
        0x00,
        (unsigned char)(count >> 8),
        (unsigned char)(count),
        0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    int size;

    rc = msc_out("SCSI: Write10 LUN: 0x00", data, 31, verbose);
    if (rc < 0) {
        return rc;
    }

    size = 512 * count;
    rc = msc_out("SCSI: Data Out LUN", buf, size, verbose);
    if (rc < 0) {
        printf("1\n");
        return rc;
    }

    size = sizeof(data);
    rc = msc_in("SCSI: Data In LUN", data, &size, verbose);
    if (rc < 0) {
        printf("1\n");
        return rc;
    }

    return rc;
}
#endif

static int send_mass_storage_command(libusb_device_handle *handle, uint8_t endpoint, uint8_t lun,
	uint8_t *cdb, uint8_t direction, int data_length, uint32_t *ret_tag)
{
	static uint32_t tag = 1;
	uint8_t cdb_len;
	int i, r, size;
	struct command_block_wrapper cbw;

	if (cdb == NULL) {
		return -1;
	}

	if (endpoint & LIBUSB_ENDPOINT_IN) {
		perr("send_mass_storage_command: cannot send command on IN endpoint\n");
		return -1;
	}

	cdb_len = cdb_length[cdb[0]];
	if ((cdb_len == 0) || (cdb_len > sizeof(cbw.CBWCB))) {
		perr("send_mass_storage_command: don't know how to handle this command (%02X, length %d)\n",
			cdb[0], cdb_len);
		return -1;
	}

	memset(&cbw, 0, sizeof(cbw));
	cbw.dCBWSignature[0] = 'U';
	cbw.dCBWSignature[1] = 'S';
	cbw.dCBWSignature[2] = 'B';
	cbw.dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw.dCBWTag = tag++;
	cbw.dCBWDataTransferLength = data_length;
	cbw.bmCBWFlags = direction;
	cbw.bCBWLUN = lun;
	// Subclass is 1 or 6 => cdb_len
	cbw.bCBWCBLength = cdb_len;
	memcpy(cbw.CBWCB, cdb, cdb_len);

	i = 0;
	do {
		// The transfer length must always be exactly 31 bytes.
		r = libusb_bulk_transfer(handle, endpoint, (unsigned char*)&cbw, 31, &size, 1000);
		if (r == LIBUSB_ERROR_PIPE) {
			libusb_clear_halt(handle, endpoint);
		}
		i++;
	} while ((r == LIBUSB_ERROR_PIPE) && (i<RETRY_MAX));
	if (r != LIBUSB_SUCCESS) {
		perr("   send_mass_storage_command: %s\n", libusb_strerror((enum libusb_error)r));
		return -1;
	}

	printf("   sent %d CDB bytes\n", cdb_len);
	return 0;
}

static int get_mass_storage_status(libusb_device_handle *handle, uint8_t endpoint, uint32_t expected_tag)
{
	int i, r, size;
	struct command_status_wrapper csw;

	// The device is allowed to STALL this transfer. If it does, you have to
	// clear the stall and try again.
	i = 0;
	do {
		r = libusb_bulk_transfer(handle, endpoint, (unsigned char*)&csw, 13, &size, 1000);
		if (r == LIBUSB_ERROR_PIPE) {
			libusb_clear_halt(handle, endpoint);
		}
		i++;
	} while ((r == LIBUSB_ERROR_PIPE) && (i<RETRY_MAX));
	if (r != LIBUSB_SUCCESS) {
		perr("   get_mass_storage_status: %s\n", libusb_strerror((enum libusb_error)r));
		return -1;
	}
	if (size != 13) {
		perr("   get_mass_storage_status: received %d bytes (expected 13)\n", size);
		return -1;
	}
	if (csw.dCSWTag != expected_tag) {
		perr("   get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",
			expected_tag, csw.dCSWTag);
		return -1;
	}
	// For this test, we ignore the dCSWSignature check for validity...
	printf("   Mass Storage Status: %02X (%s)\n", csw.bCSWStatus, csw.bCSWStatus?"FAILED":"Success");
	if (csw.dCSWTag != expected_tag)
		return -1;
	if (csw.bCSWStatus) {
		// REQUEST SENSE is appropriate only if bCSWStatus is 1, meaning that the
		// command failed somehow.  Larger values (2 in particular) mean that
		// the command couldn't be understood.
		if (csw.bCSWStatus == 1)
			return -2;	// request Get Sense
		else
			return -1;
	}

	// In theory we also should check dCSWDataResidue.  But lots of devices
	// set it wrongly.
	return 0;
}

static void get_sense(libusb_device_handle *handle, uint8_t endpoint_in, uint8_t endpoint_out)
{
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	uint8_t sense[18];
	uint32_t expected_tag;
	int size;
	int rc;

	// Request Sense
	printf("Request Sense:\n");
	memset(sense, 0, sizeof(sense));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x03;	// Request Sense
	cdb[4] = REQUEST_SENSE_LENGTH;

	send_mass_storage_command(handle, endpoint_out, 0, cdb, LIBUSB_ENDPOINT_IN, REQUEST_SENSE_LENGTH, &expected_tag);
	rc = libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&sense, REQUEST_SENSE_LENGTH, &size, 1000);
	if (rc < 0)
	{
		printf("libusb_bulk_transfer failed: %s\n", libusb_error_name(rc));
		return;
	}
	printf("   received %d bytes\n", size);

	if ((sense[0] != 0x70) && (sense[0] != 0x71)) {
		perr("   ERROR No sense data\n");
	} else {
		perr("   ERROR Sense: %02X %02X %02X\n", sense[2]&0x0F, sense[12], sense[13]);
	}
	// Strictly speaking, the get_mass_storage_status() call should come
	// before these perr() lines.  If the status is nonzero then we must
	// assume there's no data in the buffer.  For xusb it doesn't matter.
	get_mass_storage_status(handle, endpoint_in, expected_tag);
}

static libusb_device_handle *handle;
static uint8_t endpoint_in = 0, endpoint_out = 0;	// default IN and OUT endpoints

static char vid[9], pid[9], rev[5];

static double device_size;
static uint32_t max_lba, block_size;

int msc_0x12_inquiry(unsigned char lun, int verbose) {
	uint8_t buffer[64];
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	uint32_t expected_tag;
	int size;
	int i;
	// Send Inquiry
		if (verbose > 0)
	printf("\nSending Inquiry:\n");
	memset(buffer, 0, sizeof(buffer));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x12;	// Inquiry
	cdb[4] = INQUIRY_LENGTH;

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, INQUIRY_LENGTH, &expected_tag);
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, INQUIRY_LENGTH, &size, 1000));
		if (verbose > 0)
	printf("   received %d bytes\n", size);
	// The following strings are not zero terminated
	for (i=0; i<8; i++) {
		vid[i] = buffer[8+i];
		pid[i] = buffer[16+i];
		rev[i/2] = buffer[32+i/2];	// instead of another loop
	}
	vid[8] = 0;
	pid[8] = 0;
	rev[4] = 0;
		if (verbose > 0)
	printf("   VID:PID:REV \"%8s\":\"%8s\":\"%4s\"\n", vid, pid, rev);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	}
	return 0;
}

int msc_0x25_readcapacity(unsigned char lun, int verbose) {
	uint8_t buffer[64];
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	uint32_t expected_tag;
	int size;
	// Read capacity
		if (verbose > 0)
	printf("\nReading Capacity:\n");
	memset(buffer, 0, sizeof(buffer));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x25;	// Read Capacity

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, READ_CAPACITY_LENGTH, &expected_tag);
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, READ_CAPACITY_LENGTH, &size, 1000));
		if (verbose > 0)
	printf("   received %d bytes\n", size);
	max_lba = be_to_int32(&buffer[0]);
	block_size = be_to_int32(&buffer[4]);
	device_size = ((double)(max_lba+1))*block_size/(1024*1024*1024);
		if (verbose > 0)
	printf("   Max LBA: %08X, Block Size: %08X (%.2f GB)\n", max_lba, block_size, device_size);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	}
	return 0;
}

int msc_0x28_read10(unsigned char lun, int verbose, unsigned long lba, unsigned short count, unsigned char *buf) {
	int rc, size;
	uint32_t expected_tag;
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	// Send Read
    size = block_size * count;
		if (verbose > 0)
	printf("\nAttempting to read %u bytes:\n", size);
	memset(cdb, 0, sizeof(cdb));

	cdb[0] = 0x28;	// Read(10)
        cdb[2] = (uint8_t)(lba >> 24);
        cdb[3] = (uint8_t)(lba >> 16);
        cdb[4] = (uint8_t)(lba >> 8);
        cdb[5] = (uint8_t)(lba >> 0);
        cdb[7] = (uint8_t)(count >> 8);
        cdb[8] = (uint8_t)(count >> 0);

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, size, &expected_tag);
	rc = libusb_bulk_transfer(handle, endpoint_in, buf, size, &size, 5000);
		if (verbose > 0)
	printf("   READ: received %d bytes\n", size);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	} else {
		if (verbose >= 10)
		display_buffer_hex(buf, size);
		// if ((binary_dump) && ((fd = fopen(binary_name, "w")) != NULL)) {
		// 	if (fwrite(data, 1, (size_t)size, fd) != (unsigned int)size) {
		// 		perr("   unable to write binary data\n");
		// 	}
		// 	fclose(fd);
		// }
	}
    return rc;
}

int msc_0x2A_write10(unsigned char lun, int verbose, unsigned long lba, unsigned short count, unsigned char *buf) {
	int rc, size;
	uint32_t expected_tag;
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	// Send Write
    size = block_size * count;
		if (verbose > 0)
	printf("\nAttempting to write %u bytes:\n", size);
	memset(cdb, 0, sizeof(cdb));

	cdb[0] = 0x2a;	// Write(10)
        cdb[2] = (uint8_t)(lba >> 24);
        cdb[3] = (uint8_t)(lba >> 16);
        cdb[4] = (uint8_t)(lba >> 8);
        cdb[5] = (uint8_t)(lba >> 0);
        cdb[7] = (uint8_t)(count >> 8);
        cdb[8] = (uint8_t)(count >> 0);

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_OUT, size, &expected_tag);
	rc = libusb_bulk_transfer(handle, endpoint_out, buf, size, &size, 5000);
		if (verbose > 0)
	printf("   WRITE: sent %d bytes\n", size);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	} else {
		if (verbose >= 10)
		display_buffer_hex(buf, size);
		// if ((binary_dump) && ((fd = fopen(binary_name, "w")) != NULL)) {
		// 	if (fwrite(data, 1, (size_t)size, fd) != (unsigned int)size) {
		// 		perr("   unable to write binary data\n");
		// 	}
		// 	fclose(fd);
		// }
	}
    return rc;
}


#if 0
// Mass Storage device to test bulk transfers (non destructive test)
static int test_mass_storage(libusb_device_handle *handle, uint8_t endpoint_in, uint8_t endpoint_out)
{
	int r, size;
	uint8_t lun;
	uint32_t expected_tag;
	uint32_t i, max_lba, block_size;
	double device_size;
	uint8_t cdb[16];	// SCSI Command Descriptor Block
	uint8_t buffer[64];
	char vid[9], pid[9], rev[5];
	unsigned char *data;
	FILE *fd;

	printf("\nReading Max LUN:\n");
	r = libusb_control_transfer(handle, LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE,
		BOMS_GET_MAX_LUN, 0, 0, &lun, 1, 1000);
	// Some devices send a STALL instead of the actual value.
	// In such cases we should set lun to 0.
	if (r == LIBUSB_ERROR_PIPE) {
		lun = 0;
		printf("   Stalled, setting Max LUN to 0\n");
	} else if (r < 0) {
		perr("   Failed.\n");
		return r;
	} else {
		printf("   Max LUN = %d\n", lun);
	}

	// Send Inquiry
	printf("\nSending Inquiry:\n");
	memset(buffer, 0, sizeof(buffer));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x12;	// Inquiry
	cdb[4] = INQUIRY_LENGTH;

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, INQUIRY_LENGTH, &expected_tag);
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, INQUIRY_LENGTH, &size, 1000));
	printf("   received %d bytes\n", size);
	// The following strings are not zero terminated
	for (i=0; i<8; i++) {
		vid[i] = buffer[8+i];
		pid[i] = buffer[16+i];
		rev[i/2] = buffer[32+i/2];	// instead of another loop
	}
	vid[8] = 0;
	pid[8] = 0;
	rev[4] = 0;
	printf("   VID:PID:REV \"%8s\":\"%8s\":\"%4s\"\n", vid, pid, rev);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	}

	// Read capacity
	printf("\nReading Capacity:\n");
	memset(buffer, 0, sizeof(buffer));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x25;	// Read Capacity

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, READ_CAPACITY_LENGTH, &expected_tag);
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, READ_CAPACITY_LENGTH, &size, 1000));
	printf("   received %d bytes\n", size);
	max_lba = be_to_int32(&buffer[0]);
	block_size = be_to_int32(&buffer[4]);
	device_size = ((double)(max_lba+1))*block_size/(1024*1024*1024);
	printf("   Max LBA: %08X, Block Size: %08X (%.2f GB)\n", max_lba, block_size, device_size);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	}

	// coverity[tainted_data]
	data = (unsigned char*) calloc(1, block_size);
	if (data == NULL) {
		perr("   unable to allocate data buffer\n");
		return -1;
	}

	// Send Read
	printf("\nAttempting to read %u bytes:\n", block_size);
	memset(cdb, 0, sizeof(cdb));

	cdb[0] = 0x28;	// Read(10)
	cdb[8] = 0x01;	// 1 block

	send_mass_storage_command(handle, endpoint_out, lun, cdb, LIBUSB_ENDPOINT_IN, block_size, &expected_tag);
	libusb_bulk_transfer(handle, endpoint_in, data, block_size, &size, 5000);
	printf("   READ: received %d bytes\n", size);
	if (get_mass_storage_status(handle, endpoint_in, expected_tag) == -2) {
		get_sense(handle, endpoint_in, endpoint_out);
	} else {
		display_buffer_hex(data, size);
		if ((binary_dump) && ((fd = fopen(binary_name, "w")) != NULL)) {
			if (fwrite(data, 1, (size_t)size, fd) != (unsigned int)size) {
				perr("   unable to write binary data\n");
			}
			fclose(fd);
		}
	}
	free(data);

	return 0;
}
#endif

int runfs() {
    FRESULT res;
    BYTE work[FF_MAX_SS];

    memset(&fs, 0, sizeof(fs));
    memset(work, 0, sizeof(work));

    res = f_mount(&fs, "", 0);
    if (res) {
        printf("f_mount : %d\n", res);
        return res;
    }

    {
        FRESULT fr;
        FIL f;
        BYTE buffer[4096];
        UINT br;

        fr = f_open(&f, "INFO_UF2.TXT", FA_READ);
        if (fr) {
            printf("f_open2 %d\n", fr);
            return fr;
        }

        fr = f_read(&f, buffer, sizeof(buffer), &br);
        if (fr) {
            printf("f_read2 %d\n", fr);
            return fr;
        }

        printf("%s\n", buffer);

        fr = f_close(&f);
        if (fr) {
            printf("f_close2 %d\n", fr);
            return fr;
        }
    }

    {
        FRESULT fr;
        FIL f;
        BYTE *buffer;
        UINT buffer_size;
        UINT br, bw;

        {
            //char *filename = "b1_hello.uf2";
            char *filename = "b5.uf2";

            // load b1.uf2
            FILE *fp;
            errno_t err;

            struct stat st;
            err = stat(filename, &st);
            if (err) {
                printf("stat err %d\n", err);
                return err;
            }
            printf("size %ld\n", st.st_size);
            buffer = (BYTE *)malloc(st.st_size);
            buffer_size = st.st_size;


            err = fopen_s(&fp, filename, "rb");
            if (err) {
                printf("fopen_s error %d\n", err);
                return err;
            }
            fread(buffer, 1, buffer_size, fp);
            fclose(fp);
        }

        //printf("stopped\n");
        //return 1;

        printf("-- fopen\n");
        fr = f_open(&f, "output.uf2", FA_WRITE | FA_CREATE_ALWAYS);
        if (fr) {
            printf("f_open1 %d\n", fr);
            return fr;
        }

        printf("-- f_write\n");
        br = buffer_size;
        fr = f_write(&f, buffer, br, &bw);
        if (fr) {
            printf("f_write1 %d\n", fr);
            return fr;
        }

        printf("-- f_close\n");
        fr = f_close(&f);
        if (fr) {
            printf("f_close1 %d\n", fr);
            return fr;
        }

    }

    return 0;
}


int main(int argc, char **argv) {
    libusb_context *context;
    // libusb_device_handle *handle;
    libusb_device *device;
    struct libusb_device_descriptor desc;
    unsigned char buffer[256];
    int fd;
    assert((argc > 1) && (sscanf(argv[1], "%d", &fd) == 1));
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    assert(!libusb_init(&context));
    assert(!libusb_wrap_sys_device(context, (intptr_t) fd, &handle));
    device = libusb_get_device(handle);
    assert(!libusb_get_device_descriptor(device, &desc));
    printf("Vendor ID: %04x\n", desc.idVendor);
    printf("Product ID: %04x\n", desc.idProduct);
    assert(libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, buffer, 256) >= 0);
    printf("Manufacturer: %s\n", buffer);
    assert(libusb_get_string_descriptor_ascii(handle, desc.iProduct, buffer, 256) >= 0);
    printf("Product: %s\n", buffer);
    if (libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, buffer, 256) >= 0)
        printf("Serial No: %s\n", buffer);

    devh = handle;
endpoint_out = 0x02;
endpoint_in = 0x81;

    int if_num_max = 2;
    int mode = MODE;
    switch (mode) {
        case MODE_1_MONITOR:
            if_num_max = 2;
            break;
        case MODE_2_TOUCH:
            if_num_max = 2;
            break;
        default:
            if_num_max = 2;
            break;
    }

    int rc;
    for (int if_num = 0; if_num < if_num_max; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            fprintf(stderr, "Error claiming interface: %s\n", libusb_error_name(rc));
            goto out;
        }
    }

    switch (mode) {
        case MODE_1_MONITOR:
            printf("mode : monitor (1)\n");
            rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
            if (rc < 0) {
                fprintf(stderr, "Error during control transfer: %s\n", libusb_error_name(rc));
            }

            // for USBCDC : 9600bps : 0x8025
            unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };

            rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding, sizeof(encoding), 0);
            if (rc < 0) {
                fprintf(stderr, "Error during control transfer2: %s\n", libusb_error_name(rc));
            }

            // tytouf/libusb-cdc-example
            unsigned char buf[1024];
            int len;

            while (1) {
                write_chars("tinygo\r\n");
                len = read_chars(buf, 1024);
                buf[len] = 0;
                fprintf(stdout, "%s", buf);
                usleep(10 * 1000);
            }
            break;

        case MODE_2_TOUCH:
            printf("mode : touch (2)\n");
            rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
            if (rc < 0) {
                fprintf(stderr, "Error during control transfer: %s\n", libusb_error_name(rc));
            }

            // for enter boot loader : 1200bps : 0x04B0
            unsigned char encoding2[] = { 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08 };
            rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding2, sizeof(encoding2), 0);
            if (rc < 0) {
                fprintf(stderr, "Error during control transfer2: %s\n", libusb_error_name(rc));
            }

            // ignore errors
            libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_RTS, 0, NULL, 0, 0);
            libusb_control_transfer(devh, 0x21, 0x22, 0, 0, NULL, 0, 0);

            break;

        default:
            // flash
            printf("mode : flash (0)\n");
            msc_0x12_inquiry(0, 10);
            msc_0x25_readcapacity(0, 10);
            {
                //unsigned char buf[512];
                //msc_0x28_read10(0, 10, 0, 1, &buf[0]);

                rc = runfs();
                if (rc) {
                    printf("runfs error %d\n", rc);
                }
            }
            //sleep(1);
            //msc_0x28_read10(0, 10, 1);
            break;
    }

out:
    libusb_exit(context);
}

DWORD get_fattime(void) {
    time_t t;
    struct tm *stm;


    t = time(0);
    stm = localtime(&t);

    return (DWORD)(stm->tm_year - 80) << 25 |
        (DWORD)(stm->tm_mon + 1) << 21 |
        (DWORD)stm->tm_mday << 16 |
        (DWORD)stm->tm_hour << 11 |
        (DWORD)stm->tm_min << 5 |
        (DWORD)stm->tm_sec >> 1;
}
