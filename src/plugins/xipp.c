#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <eegdev-pluginapi.h>
#include <float.h>

#include "xipp_wrapper.h"

struct xipp_eegdev {
    struct devmodule dev;

	unsigned int NUM_EEG_CH;
	unsigned int* EEG_CH_LIST;

	unsigned int NUM_SENSOR_CH;
	unsigned int* SENSOR_CH_LIST;

	unsigned int NUM_CH;
	unsigned int* CH_LIST;
	unsigned int* CH_MAP;

	unsigned int offset[EGD_NUM_STYPE];

    pthread_t thread_id;
    pthread_mutex_t acqlock;
    unsigned int runacq;
};

#define get_xipp(dev_p) ((struct xipp_eegdev*)(dev_p))

#define IS_EEG 1
#define IS_SIGNAL 2

static const char xippunit[] = "uV";
static const char xipptransducter[] = "Electrode";

static const union gval eego_scales[EGD_NUM_DTYPE] = {
    [EGD_INT32] = {.valint32_t = 1},
    [EGD_FLOAT] = {.valfloat = 1.0f},  // in uV
    [EGD_DOUBLE] = {.valdouble = 1000000}    // in uV
};

enum {
	USE_TCP,
	EEG_MASK,
	SENSOR_MASK,
	STREAM,
	NUMOPT
};
static const struct egdi_optname xipp_options[] = {
    [USE_TCP] = {.name = "TCP", .defvalue = "1"},
	[EEG_MASK] = {.name="EEG_MASK", .defvalue = "0xffffffff000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"},
	[SENSOR_MASK] = {.name="SENSOR_MASK", .defvalue = "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"},
	[STREAM] = {.name = "STREAM", .defvalue = "hi-res"},
	[NUMOPT] = {.name = NULL}};

static void* xipp_read_fn(void* arg)
{
	struct xipp_eegdev* xippdev = arg;
	const struct core_interface* restrict ci = &xippdev->dev.ci;

	// for (int i = 0; i < xippdev->NUM_CH; i++) {
	// 	printf("%d, ", xippdev->CH_LIST[i]);
	// }
	// printf("\n");

	int num_points = 32;

	float* data_out = (float*) malloc(sizeof(float) * num_points * xippdev->NUM_CH);
	unsigned int* ts_out = (unsigned int*) malloc(sizeof(unsigned int) * 1);
    ts_out[0] = 1;

	int bytes_to_allocate = sizeof(float) * (xippdev->NUM_CH);
	float* buffer = (float*) malloc(bytes_to_allocate);

	unsigned int prev_ts = xl_time();

	float reference_point = 0.0;

	while (1) {
		if (xippdev->runacq == 0) {
			break;
		}
		// Get 32 points and the new timestamp
		unsigned int n_points = xl_cont_hires(ts_out, data_out, num_points, xippdev->CH_LIST, xippdev->NUM_CH, 0);
		if (n_points < (num_points - 1)) {
			printf("%d connected channels, %d points, %d new points\n", xippdev->NUM_CH, num_points, n_points);
			printf("Error getting data\n");
			goto error;
		}
		if (prev_ts == ts_out[0]) {
			usleep(50);
			continue;
		}
		// Divide difference in timestamps by 4 to get number of new points
		// int num_new_points = (ts_out[0] - prev_ts) / 4;
		// if (num_new_points > 32) {
		// 	num_new_points = 32;
		// 	printf("Too many new points, buffer is skipping\n");
		// }
		// printf("%d difference between timestamps\n", num_new_points);
		int num_new_points = n_points;
		for (int i = 0; i < num_points; i++) {
			if (reference_point == data_out[i]) {
				// printf("%d new datapoints\n", num_points - (i + 1));
				num_new_points = (num_points - (i + 1));
				break;
			}
		}
		for (int i = (num_points - num_new_points); i < num_points; i++) {
			unsigned int eeg_count = 0;
			unsigned int signal_count = 0;
			// printf("[");
			for (int j = 0; j < xippdev->NUM_CH; j++) {
				// printf("%d\n", data_out[j * num_points + i]);
				// switch(xippdev->CH_MAP[xippdev->CH_LIST[j]]) {
				// 	case IS_EEG:
				// 		// printf(" %d is EEG ", xippdev->CH_LIST[j]);
				// 		buffer[eeg_count] = data_out[j * num_points + i];
				// 		eeg_count++;
				// 		break;
				// 	case IS_SIGNAL:
				// 	// printf(" %d is Signal ", xippdev->CH_LIST[j]);
				// 		buffer[xippdev->NUM_EEG_CH + signal_count] = data_out[j * num_points + i];
				// 		signal_count++;
				// 		break;
				// 	default: 
				// 		break;
				// }
				if (j == 0) {
					reference_point = data_out[i];
				}
				buffer[j] = data_out[j * num_points + i];
			}
			// printf("]\n");
			// buffer[xippdev->NUM_CH] = 0;
			// printf("\n");
			ci->update_ringbuffer(&(xippdev->dev), buffer, bytes_to_allocate);
			// usleep(50);
		}

		prev_ts = ts_out[0];
	}

	free(data_out);
	free(ts_out);
	free(buffer);

	return NULL;
error:
	free(data_out);
	free(ts_out);
	free(buffer);
	ci->report_error(&(xippdev->dev), EIO);
	return NULL;
}

/**
 * @brief      Sets the cap's capabilities.
 *
 * @param      xippdev  The device's structure.
 * @param      optv     The optv.
 *
 * @return     Always 0.
 */
static int xipp_set_capability(struct xipp_eegdev* xippdev,
                               const char* optv[]) {
  struct systemcap cap = {
      .sampling_freq = 2000,
      .type_nch[EGD_EEG] = xippdev->NUM_EEG_CH,
      .type_nch[EGD_SENSOR] = xippdev->NUM_SENSOR_CH,
      .type_nch[EGD_TRIGGER] = 0,
      .device_type = "Xipp (Ripple)",
      .device_id = "Macro + Stim"
  };
  struct devmodule* dev = &xippdev->dev;

  xippdev->offset[EGD_EEG] = 0;
  xippdev->offset[EGD_SENSOR] = xippdev->NUM_EEG_CH * sizeof(float);
  xippdev->offset[EGD_TRIGGER] = xippdev->offset[EGD_SENSOR] + (xippdev->NUM_SENSOR_CH) * sizeof(float);
  
  dev->ci.set_cap(dev, &cap);
  //dev->ci.set_input_samlen(dev, (eegodev->NCH ) * sizeof(double) - 1);
  dev->ci.set_input_samlen(dev, (xippdev->NUM_CH + 1 ) * sizeof(float));
  return 0;
}

// Function to convert a single hex character to its decimal value
unsigned int hexchar_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0;
}

// Function to generate and print a list of positions of set bits
unsigned int generate_set_bit_positions(const char* mask, unsigned int* list_ptr) {
    int count = 0;
    if (mask[0] == '0' && (mask[1] == 'x' || mask[1] == 'X')) mask += 2;
    for (int i = 0; i < 128; i++) {
        unsigned int num = hexchar_to_int(mask[i]);
        for (int j = 0; j < 4; j++) {
            if (num && (1 << j)) {
                list_ptr[count] = i * 4 + j;
				count++;
            }
        }
    }
    return count;
}

/******************************************************************
 *               xipp/Ripple methods implementation                	  *
 ******************************************************************/
static
int xipp_open_device(struct devmodule* dev, const char* optv[])
{

	printf("Opened XIPP library\n");
	struct xipp_eegdev* xippdev = get_xipp(dev);

	bool use_tcp = (strcmp(optv[USE_TCP], "true") == 0 || strcmp(optv[USE_TCP], "1") == 0);

    if (use_tcp) {
        printf("Using TCP\n");
        if (xl_open_tcp() < 0) {
            printf("Failed to open TCP connection\n");
            return -1;
        } else {
			printf("Connected to device\n");
		}
    } else {
        printf("Using UDP\n");
        if (xl_open_udp() < 0) {
            printf("Failed to open UDP connection\n");
            return -1;
        } else {
			printf("Connected to device\n");
		}
    }

	// Get channels
	xippdev->CH_MAP = (unsigned int*) malloc(sizeof(unsigned int) * 512);
	memset(xippdev->CH_MAP, 0, sizeof(unsigned int) * 512);

	xippdev->EEG_CH_LIST = (unsigned int*) malloc(sizeof(unsigned int) * 512);
	xippdev->NUM_EEG_CH = generate_set_bit_positions(optv[EEG_MASK], xippdev->EEG_CH_LIST);

	printf("%d EEG Channels Specified\n", xippdev->NUM_EEG_CH);

	for (int i = 0; i < xippdev->NUM_EEG_CH; i++) {
		xippdev->CH_MAP[xippdev->EEG_CH_LIST[i]] = IS_EEG;
	}

	printf("%s\n", optv[SENSOR_MASK]);

	xippdev->SENSOR_CH_LIST = (unsigned int*) malloc(sizeof(unsigned int) * 512);
	xippdev->NUM_SENSOR_CH = generate_set_bit_positions(optv[SENSOR_MASK], xippdev->SENSOR_CH_LIST);

	printf("%d Sensor Channels Specified\n", xippdev->NUM_SENSOR_CH);

	for (int i = 0; i < xippdev->NUM_SENSOR_CH; i++) {
		xippdev->CH_MAP[xippdev->SENSOR_CH_LIST[i]] = IS_SIGNAL;
	}

	// Check for duplicates
	for (int i = 0; i < xippdev->NUM_EEG_CH; i++) {
		for (int j = 0; j < xippdev->NUM_SENSOR_CH; j++) {
			if (xippdev->EEG_CH_LIST[i] == xippdev->SENSOR_CH_LIST[j]) {
				printf("Duplicate value for EEG and Sensor\n");
				return -1;
			}
		}
	}

	xippdev->NUM_CH = xippdev->NUM_EEG_CH + xippdev->NUM_SENSOR_CH;

	xippdev->CH_LIST = (unsigned int*) malloc(sizeof(unsigned int) * xippdev->NUM_CH);
	unsigned int count = 0;
	for (int i = 0; i < 512; i++) {
		if (xippdev->CH_MAP[i] != 0) {
			xippdev->CH_LIST[count] = i;
			count++;
		}
	}

	if (xippdev->NUM_CH == 0 || xippdev->NUM_CH > 512) {
		printf("Invalid number of electrodes\n");
		return -1;
	}

	int max_elecs = 512;
	unsigned int elecs_out[max_elecs];
	memset(elecs_out, 0, sizeof(elecs_out));

	unsigned int* connected_channels = elecs_out;

	int n = xl_list_elec(connected_channels, max_elecs, "");
	if (n < 0) {
		printf("Failed to list electrodes\n");
		return -1;
	}
	if (n < xippdev->NUM_CH) {
		printf("Invalid number of electrodes\n");
		return -1;
	}
	for (int i = 0; i < xippdev->NUM_EEG_CH; i++) {
		bool is_contained = false;
		for (int j = 0; j < n; j++) {
			if (xippdev->EEG_CH_LIST[i] == connected_channels[j]) {
				is_contained = true;
				break;
			}
		}
		if (!is_contained) {
			printf("EEG electrode %d is not connected\n", xippdev->EEG_CH_LIST[i]);
			return -1;
		}
	}
	for (int i = 0; i < xippdev->NUM_SENSOR_CH; i++) {
		bool is_contained = false;
		for (int j = 0; j < n; j++) {
			if (xippdev->SENSOR_CH_LIST[i] == connected_channels[j]) {
				is_contained = true;
				break;
			}
		}
		if (!is_contained) {
			printf("Sensor electrode %d is not connected\n", xippdev->SENSOR_CH_LIST[i]);
			return -1;
		}
	}

	char* stream = optv[STREAM];
	int max_streams = 512;
	char streams_out[max_streams];
	memset(streams_out, 0, sizeof(streams_out));

	char* streams = streams_out;

	n = xl_get_fe_streams(streams, max_streams, 0);

	if (n < 0) {
		printf("Failed to get streams\n");
		return -1;
	}

	int has_stream = 0;
	for (int i = 0; i < n; i++) {
		if (strcmp(streams + i * STRLEN_LABEL, stream) == 0) {
			has_stream = 1;
			break;
		}
	}
	if (!has_stream) {
		printf("Stream %s not found\n", stream);
		return -1;
	}

	// Set channels to correct stream
	for (int i = 0; i < n; i++) {
		if (xl_signal_set(connected_channels[i], stream, 1) < 0) {
			printf("Failed to set stream\n");
			return -1;
		}
	}

	xipp_set_capability(xippdev, optv);

	pthread_mutex_init(&xippdev->acqlock, NULL);
	xippdev->runacq = 1;
	int ret;
	if ((ret = pthread_create(&(xippdev->thread_id), NULL, 
								xipp_read_fn, xippdev)))
	  goto error;

	return 0;

error:
	return -1;
}


static
int xipp_close_device(struct devmodule* dev)
{
	struct xipp_eegdev* xippdev = get_xipp(dev);

	pthread_mutex_lock(&xippdev->acqlock);
    xippdev->runacq = 0;
    pthread_mutex_unlock(&xippdev->acqlock);

	free_label(xippdev);
	pthread_join(xippdev->thread_id, NULL);
	pthread_mutex_destroy(&xippdev->acqlock);
	xl_close();

    return 0;
}

// Hardcoded for now
static
int xipp_set_channel_groups(struct devmodule* dev, unsigned int ngrp,
                            const struct grpconf* grp)
{
	unsigned int i, stype;
	struct selected_channels* selch;
	struct xipp_eegdev* xippdev = get_xipp(dev);
	
	if (!(selch = dev->ci.alloc_input_groups(dev, ngrp))) return -1;

	for (i = 0; i < ngrp; i++) {
		stype = grp[i].sensortype;
		// Set parameters of (eeg -> ringbuffer)
		selch[i].in_offset = grp[i].index * sizeof(float);
		selch[i].inlen = grp[i].nch * sizeof(float);
		selch[i].bsc = (stype == EGD_TRIGGER) ? 0 : 1;
		selch[i].typein = EGD_FLOAT;
		selch[i].sc = eego_scales[grp[i].datatype];
		selch[i].typeout = grp[i].datatype;
		selch[i].iarray = grp[i].iarray;
		selch[i].arr_offset = grp[i].arr_offset;
	}

    return 0;
}

// Hardcoded for now
static
void xipp_fill_chinfo(const struct devmodule* dev, int stype,
                      unsigned int ich, struct egd_chinfo* info)
{
	struct xipp_eegdev* xippdev = get_xipp(dev);

	if (stype != EGD_TRIGGER) {
		info->isint = 0;
		info->dtype = EGD_FLOAT;
		info->min.valdouble = -187500.0;
		info->max.valdouble = 187500.0;
		info->label = (stype == EGD_EEG) ? "EEG" : "Sensor";
		info->unit = xippunit;
		info->transducter = xipptransducter;
	} else {
		info->isint = 0;
		info->dtype = EGD_DOUBLE;
		info->min.valdouble = -DBL_MAX;
		info->max.valdouble = DBL_MAX;
		info->label = "Status";
		info->unit = "Boolean";
		info->transducter = "Trigger";
	}
}

API_EXPORTED
const struct egdi_plugin_info eegdev_plugin_info = {
    .plugin_abi = EEGDEV_PLUGIN_ABI_VERSION,
    .struct_size = sizeof(struct xipp_eegdev),
    .open_device = xipp_open_device,
    .close_device = xipp_close_device,
    .set_channel_groups = xipp_set_channel_groups,
    .fill_chinfo = xipp_fill_chinfo,
    .supported_opts = xipp_options};
