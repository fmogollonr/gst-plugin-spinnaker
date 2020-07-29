/* GStreamer Spinnaker Plugin
 * Copyright (C) 2019 Embry-Riddle Aeronautical University
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 *
 * Author: D Thompson
 *
 */
/**
 * SECTION:element-GstSpinnaker_src
 *
 * The spinnakersrc element is a source for a USB 3 camera supported by the FLIR Spinnaker SDK.
 * A live source, operating in push mode.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 spinnakersrc ! videoconvert ! autovideosink
 * ]|
 * </refsect2>
 */

// Which functions of the base class to override. Create must alloc and fill the buffer. Fill just needs to fill it
//#define OVERRIDE_FILL  !!! NOT IMPLEMENTED !!!
#define OVERRIDE_CREATE

#include <unistd.h> // for usleep
#include <string.h> // for memcpy
#include <math.h>  // for pow
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <gst/video/video.h>

#include "gstspinnaker.h"

GST_DEBUG_CATEGORY_STATIC (gst_spinnaker_src_debug);
#define GST_CAT_DEFAULT gst_spinnaker_src_debug

/* prototypes */
static void gst_spinnaker_src_set_property (GObject * object,
		guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_spinnaker_src_get_property (GObject * object,
		guint property_id, GValue * value, GParamSpec * pspec);
static void gst_spinnaker_src_dispose (GObject * object);
static void gst_spinnaker_src_finalize (GObject * object);

static gboolean gst_spinnaker_src_start (GstBaseSrc * src);
static gboolean gst_spinnaker_src_stop (GstBaseSrc * src);
static GstCaps *gst_spinnaker_src_get_caps (GstBaseSrc * src, GstCaps * filter);
static gboolean gst_spinnaker_src_set_caps (GstBaseSrc * src, GstCaps * caps);

#ifdef OVERRIDE_CREATE
	static GstFlowReturn gst_spinnaker_src_create (GstPushSrc * src, GstBuffer ** buf);
#endif
#ifdef OVERRIDE_FILL
	static GstFlowReturn gst_spinnaker_src_fill (GstPushSrc * src, GstBuffer * buf);
#endif

//static GstCaps *gst_spinnaker_src_create_caps (GstSpinnakerSrc * src);
static void gst_spinnaker_src_reset (GstSpinnakerSrc * src);
enum
{
	PROP_0,
	PROP_CAMERA,
	PROP_WIDTH,
	PROP_HEIGHT
};

#define	FLYCAP_UPDATE_LOCAL  FALSE
#define	FLYCAP_UPDATE_CAMERA TRUE

#define DEFAULT_PROP_CAMERA	           0
#define DEFAULT_PROP_EXPOSURE           40.0
#define DEFAULT_PROP_GAIN               1
#define DEFAULT_PROP_BLACKLEVEL         15
#define DEFAULT_PROP_RGAIN              425
#define DEFAULT_PROP_BGAIN              727
#define DEFAULT_PROP_BINNING            1
#define DEFAULT_PROP_SHARPNESS			2    // this is 'normal'
#define DEFAULT_PROP_SATURATION			50   // this is 100 on the camera scale 0-400
#define DEFAULT_PROP_HORIZ_FLIP         0
#define DEFAULT_PROP_VERT_FLIP          0
#define DEFAULT_PROP_WHITEBALANCE       GST_WB_MANUAL
#define DEFAULT_PROP_LUT		        GST_LUT_1
#define DEFAULT_PROP_LUT1_OFFSET		0    
#define DEFAULT_PROP_LUT1_GAMMA		    0.45
#define DEFAULT_PROP_LUT1_GAIN		    1.099
#define DEFAULT_PROP_LUT2_OFFSET		10    
#define DEFAULT_PROP_LUT2_GAMMA		    0.45
#define DEFAULT_PROP_LUT2_GAIN		    1.501   
#define DEFAULT_PROP_MAXFRAMERATE       25
#define DEFAULT_PROP_GAMMA			    1.5
#define DEFAULT_PROP_WIDTH 				640
#define DEFAULT_PROP_HEIGHT			    512

#define DEFAULT_GST_VIDEO_FORMAT GST_VIDEO_FORMAT_GRAY8
// Put matching type text in the pad template below

// pad template
static GstStaticPadTemplate gst_spinnaker_src_template =
		GST_STATIC_PAD_TEMPLATE ("src",
				GST_PAD_SRC,
				GST_PAD_ALWAYS,
				GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE
						("{ GRAY8 }"))
		);

#define EXEANDCHECK(function) \
{\
	spinError Ret = function;\
	if (SPINNAKER_ERR_SUCCESS != Ret){\
		GST_ERROR_OBJECT(src, "Spinnaker call failed: %d", Ret);\
		goto fail;\
	}\
}

G_DEFINE_TYPE_WITH_CODE (GstSpinnakerSrc, gst_spinnaker_src, GST_TYPE_PUSH_SRC,
    GST_DEBUG_CATEGORY_INIT (GST_CAT_DEFAULT, "spinnaker", 0,
        "debug category for spinnaker element"));

// This function handles the error prints when a node or entry is unavailable or
// not readable/writable on the connected camera
void PrintRetrieveNodeFailure(char node[], char name[])
{
    printf("Unable to get %s (%s %s retrieval failed).\n\n", node, name, node);
}

// This function helps to check if a node is available and readable
bool8_t IsAvailableAndReadable(spinNodeHandle hNode, char nodeName[])
{
    bool8_t pbAvailable = False;
    spinError err = SPINNAKER_ERR_SUCCESS;
    err = spinNodeIsAvailable(hNode, &pbAvailable);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to retrieve node availability (%s node), with error %d...\n\n", nodeName, err);
    }

    bool8_t pbReadable = False;
    err = spinNodeIsReadable(hNode, &pbReadable);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to retrieve node readability (%s node), with error %d...\n\n", nodeName, err);
    }
    return pbReadable && pbAvailable;
}

// This function helps to check if a node is available and writable
bool8_t IsAvailableAndWritable(spinNodeHandle hNode, char nodeName[])
{
    bool8_t pbAvailable = False;
    spinError err = SPINNAKER_ERR_SUCCESS;
    err = spinNodeIsAvailable(hNode, &pbAvailable);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to retrieve node availability (%s node), with error %d...\n\n", nodeName, err);
    }

    bool8_t pbWritable = False;
    err = spinNodeIsWritable(hNode, &pbWritable);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to retrieve node writability (%s node), with error %d...\n\n", nodeName, err);
    }
    return pbWritable && pbAvailable;
}

// This function configures a number of settings on the camera including
// offsets X and Y, width, height, and pixel format. These settings will be
// applied before spinCameraBeginAcquisition() is called; otherwise, they will
// be read only. Also, it is important to note that settings are applied
// immediately. This means if you plan to reduce the width and move the x
// offset accordingly, you need to apply such changes in the appropriate order.
spinError ConfigureCustomImageSettings(spinNodeMapHandle hNodeMap)
{
    spinError err = SPINNAKER_ERR_SUCCESS;

    printf("\n\n*** CONFIGURING CUSTOM IMAGE SETTINGS ***\n\n");

    //
    // Apply mono 8 pixel format
    //
    // *** NOTES ***
    // Enumeration nodes are slightly more complicated to set than other
    // nodes. This is because setting an enumeration node requires working
    // with two nodes instead of the usual one.
    //
    // As such, there are a number of steps to setting an enumeration node:
    // retrieve the enumeration node from the nodemap, retrieve the desired
    // entry node from the enumeration node, retrieve the integer value
    // from the entry node, and set the new value of the enumeration node
    // with the integer value from the entry node.
    //
    // It is important to note that there are two sets of functions that might
    // produce erroneous results if they were to be mixed up. The first two
    // functions, spinEnumerationSetIntValue() and
    // spinEnumerationEntryGetIntValue(), use the integer values stored on each
    // individual cameras. The second two, spinEnumerationSetEnumValue() and
    // spinEnumerationEntryGetEnumValue(), use enum values defined in the
    // Spinnaker library. The int and enum values will most likely be
    // different from another.
    //
    spinNodeHandle hPixelFormat = NULL;
    spinNodeHandle hPixelFormatMono8 = NULL;
    int64_t pixelFormatMono8 = 0;

    // Retrieve enumeration node from the nodemap
    err = spinNodeMapGetNode(hNodeMap, "PixelFormat", &hPixelFormat);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set pixel format (node retrieval). Aborting with error %d...\n\n", err);
        return err;
    }

    // Retrieve desired entry node from the enumeration node
    if (IsAvailableAndReadable(hPixelFormat, "PixelFormat"))
    {
        err = spinEnumerationGetEntryByName(hPixelFormat, "Mono14", &hPixelFormatMono8);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to set pixel format (enum entry retrieval). Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "PixelFormat");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // Retrieve integer value from entry node
    if (IsAvailableAndReadable(hPixelFormatMono8, "PixelFormatMono14"))
    {
        err = spinEnumerationEntryGetIntValue(hPixelFormatMono8, &pixelFormatMono8);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to set pixel format (enum entry int value retrieval). Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("entry", "PixelFormat 'Mono14'");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // Set integer as new value for enumeration node
    if (IsAvailableAndWritable(hPixelFormat, "PixelFormat"))
    {
        err = spinEnumerationSetIntValue(hPixelFormat, pixelFormatMono8);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to set pixel format (enum entry setting). Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "PixelFormat");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    printf("Pixel format set to 'mono10'...\n");

    //
    // Apply minimum to offset X
    //
    // *** NOTES ***
    // Numeric nodes have both a minimum and maximum. A minimum is retrieved
    // with the method GetMin(). Sometimes it can be important to check
    // minimums to ensure that your desired value is within range.
    //
    // Notice that the node type is explicitly expressed in the name of the
    // second and third functions. Although node types are not expressed in
    // node handles, knowing the node type is important to interacting with
    // a node in any meaningful way.
    //
    spinNodeHandle hOffsetX = NULL;
    int64_t offsetXMin = 0;

    err = spinNodeMapGetNode(hNodeMap, "OffsetX", &hOffsetX);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set offset x. Aborting with error %d...\n\n", err);
        return err;
    }

    // get min
    if (IsAvailableAndWritable(hOffsetX, "OffsetX"))
    {
        err = spinIntegerGetMin(hOffsetX, &offsetXMin);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to set offset x. Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "OffsetX");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // set min
    err = spinIntegerSetValue(hOffsetX, offsetXMin);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set offset x. Aborting with error %d...\n\n", err);
        return err;
    }
    printf("Offset X set to %d...\n", (int)offsetXMin);

    //
    // Apply minimum to offset Y
    //
    // *** NOTES ***
    // It is often desirable to check the increment as well. The increment
    // is a number of which a desired value must be a multiple. Certain
    // nodes, such as those corresponding to offsets X and Y, have an
    // increment of 1, which basically means that any value within range
    // is appropriate. The increment is retrieved with the method
    // spinIntegerGetInc().
    //
    // The offsets both hold integer values. As such, if a double were input
    // as an argument or if a string function were used, problems would
    // occur.
    //
    spinNodeHandle hOffsetY = NULL;
    int64_t offsetYMin = 0;

    err = spinNodeMapGetNode(hNodeMap, "OffsetY", &hOffsetY);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set offset Y. Aborting with error %d...\n\n", err);
        return err;
    }

    // get min
    if (IsAvailableAndWritable(hOffsetY, "OffsetY"))
    {
        err = spinIntegerGetMin(hOffsetY, &offsetYMin);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to set offset Y. Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "OffsetY");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // set min
    err = spinIntegerSetValue(hOffsetY, offsetYMin);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set offset Y. Aborting with error %d...\n\n", err);
        return err;
    }
    printf("Offset Y set to %d...\n", (int)offsetYMin);

    //
    // Set maximum width
    //
    // *** NOTES ***
    // Other nodes, such as those corresponding to image width and height,
    // might have an increment other than 1. In these cases, it can be
    // important to check that the desired value is a multiple of the
    // increment.
    //
    // This is often the case for width and height nodes. However, because
    // these nodes are being set to their maximums, there is no real reason
    // to check against the increment.
    //
    spinNodeHandle hWidth = NULL;
    int64_t widthToSet = 0;

    // Retrieve width node
    err = spinNodeMapGetNode(hNodeMap, "Width", &hWidth);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set width. Aborting with error %d...\n\n", err);
        return err;
    }

    // Retrieve maximum width
    if (IsAvailableAndWritable(hWidth, "Width"))
    {
        err = spinIntegerGetMax(hWidth, &widthToSet);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to get width. Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "Width");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // Set width
    err = spinIntegerSetValue(hWidth, widthToSet);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set width. Aborting with error %d...\n\n", err);
        return err;
    }

    printf("Width set to %d...\n", (int)widthToSet);

    //
    // Set maximum height
    //
    // *** NOTES ***
    // A maximum is retrieved with the method spinIntegerGetMax(). A node's
    // minimum and maximum should always be multiples of the increment.
    //
    spinNodeHandle hHeight = NULL;
    int64_t HeightToSet = 0;

    // Retrieve Height node
    err = spinNodeMapGetNode(hNodeMap, "Height", &hHeight);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set Height. Aborting with error %d...\n\n", err);
        return err;
    }

    // Retrieve maximum Height
    if (IsAvailableAndWritable(hHeight, "Height"))
    {
        err = spinIntegerGetMax(hHeight, &HeightToSet);
        if (err != SPINNAKER_ERR_SUCCESS)
        {
            printf("Unable to get Height. Aborting with error %d...\n\n", err);
            return err;
        }
    }
    else
    {
        PrintRetrieveNodeFailure("node", "Height");
        return SPINNAKER_ERR_ACCESS_DENIED;
    }

    // Set Height
    err = spinIntegerSetValue(hHeight, HeightToSet);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to set Height. Aborting with error %d...\n\n", err);
        return err;
    }

    printf("Height set to %d...\n", (int)HeightToSet);

    return err;
}




/* class initialisation */
static void
gst_spinnaker_src_class_init (GstSpinnakerSrcClass * klass)
{
	GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
	GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
	GstBaseSrcClass *gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
	GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);

	GST_DEBUG_CATEGORY_INIT (GST_CAT_DEFAULT, "gstspinnakersrc", 0,
			"Spinnaker Camera source");

	gobject_class->set_property = gst_spinnaker_src_set_property;
	gobject_class->get_property = gst_spinnaker_src_get_property;
	gobject_class->dispose = gst_spinnaker_src_dispose;
	gobject_class->finalize = gst_spinnaker_src_finalize;

	gst_element_class_add_pad_template (gstelement_class,
			gst_static_pad_template_get (&gst_spinnaker_src_template));

	gst_element_class_set_static_metadata (gstelement_class,
			"Spinnaker Video Source", "Source/Video",
			"Spinnaker Camera video source", "David Thompson <dave@republicofdave.net>");

	gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_spinnaker_src_start);
	gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_spinnaker_src_stop);
	gstbasesrc_class->get_caps = GST_DEBUG_FUNCPTR (gst_spinnaker_src_get_caps);
	gstbasesrc_class->set_caps = GST_DEBUG_FUNCPTR (gst_spinnaker_src_set_caps);

#ifdef OVERRIDE_CREATE
	gstpushsrc_class->create = GST_DEBUG_FUNCPTR (gst_spinnaker_src_create);
	GST_DEBUG ("Using gst_spinnaker_src_create.");
#endif
#ifdef OVERRIDE_FILL
	gstpushsrc_class->fill   = GST_DEBUG_FUNCPTR (gst_spinnaker_src_fill);
	GST_DEBUG ("Using gst_spinnaker_src_fill.");
#endif
	//camera id property
	g_object_class_install_property (gobject_class, PROP_CAMERA,
		g_param_spec_int("camera-id", "Camera ID", "Camera ID to open.", 0,7, DEFAULT_PROP_CAMERA,
		 (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS | GST_PARAM_MUTABLE_PLAYING)));
}

static void
init_properties(GstSpinnakerSrc * src)
{
    //Hardcoded hot-garbage **************************
  src->nWidth = DEFAULT_PROP_WIDTH;
  src->nHeight = DEFAULT_PROP_HEIGHT;
  src->nBytesPerPixel = 1;
  src->binning = 1;
  src->n_frames = 0;
  src->framerate = 31;
  src->last_frame_time = 0;
  src->nPitch = src->nWidth * src->nBytesPerPixel;
  src->gst_stride = src->nPitch;
  src->cameraID = DEFAULT_PROP_CAMERA;
  src->exposure = DEFAULT_PROP_EXPOSURE;

}

static void
gst_spinnaker_src_init (GstSpinnakerSrc * src)
{
	/* set source as live (no preroll) */
	gst_base_src_set_live (GST_BASE_SRC (src), TRUE);

	/* override default of BYTES to operate in time mode */
	gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME);

	init_properties(src);

	gst_spinnaker_src_reset (src);
}

static void
gst_spinnaker_src_reset (GstSpinnakerSrc * src)
{
	src->n_frames = 0;
	src->total_timeouts = 0;
	src->last_frame_time = 0;
	src->cameraID = 0;
	src->hCameraList = NULL;
	src->cameraPresent = FALSE;
	src->hSystem = NULL;
}

void
gst_spinnaker_src_set_property (GObject * object, guint property_id,
		const GValue * value, GParamSpec * pspec)
{
	GstSpinnakerSrc *src;

	src = GST_SPINNAKER_SRC (object);

	spinImage hCamera = NULL;
	spinNodeMapHandle hNodeMap = NULL;
  	EXEANDCHECK(spinCameraListGet(src->hCameraList, src->cameraID, &hCamera));
	EXEANDCHECK(spinCameraGetNodeMap(hCamera, &hNodeMap));
	spinNodeHandle hWidth = NULL;
	int64_t maxWidth = 0;
	spinNodeHandle hHeight = NULL;
	int64_t maxHeight = 0;
	switch(property_id) {
	case PROP_CAMERA:
		src->cameraID = g_value_get_int (value);
		GST_DEBUG_OBJECT (src, "camera id: %d", src->cameraID);
		break;
	case PROP_WIDTH:

		EXEANDCHECK(spinNodeMapGetNode(hNodeMap, "Width", &hWidth));

		// Retrieve maximum width
		if (IsAvailableAndWritable(hWidth, "Width"))
		{
			EXEANDCHECK(spinIntegerGetMax(hWidth, &maxWidth));
			int64_t param = g_value_get_int(value);
			if (param > maxWidth){
				EXEANDCHECK(spinIntegerSetValue(hWidth, maxWidth));
				src->nWidth = maxWidth;
			}
			else {
				EXEANDCHECK(spinIntegerSetValue(hWidth, param));
				src->nWidth = param;
			}
		}
		break;
	case PROP_HEIGHT:

		EXEANDCHECK(spinNodeMapGetNode(hNodeMap, "Height", &hHeight));

		// Retrieve maximum width
		if (IsAvailableAndWritable(hHeight, "Height"))
		{
			EXEANDCHECK(spinIntegerGetMax(hHeight, &maxHeight));
			int64_t param = g_value_get_int(value);
			if (param > maxHeight){
				EXEANDCHECK(spinIntegerSetValue(hHeight, maxHeight));
				src->nHeight = maxHeight;
			}
			else {
				EXEANDCHECK(spinIntegerSetValue(hWidth, param));
				src->nHeight = param;
			}
		}
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
		break;
	}

	fail:
	return;
}

void
gst_spinnaker_src_get_property (GObject * object, guint property_id,
		GValue * value, GParamSpec * pspec)
{
	GstSpinnakerSrc *src;

	g_return_if_fail (GST_IS_SPINNAKER_SRC (object));
	src = GST_SPINNAKER_SRC (object);
}

void
gst_spinnaker_src_dispose (GObject * object)
{
	GstSpinnakerSrc *src;

	g_return_if_fail (GST_IS_SPINNAKER_SRC (object));
	src = GST_SPINNAKER_SRC (object);

	GST_DEBUG_OBJECT (src, "dispose");

	// clean up as possible.  may be called multiple times

	G_OBJECT_CLASS (gst_spinnaker_src_parent_class)->dispose (object);
}

void
gst_spinnaker_src_finalize (GObject * object)
{
	GstSpinnakerSrc *src;

	g_return_if_fail (GST_IS_SPINNAKER_SRC (object));
	src = GST_SPINNAKER_SRC (object);

	GST_DEBUG_OBJECT (src, "finalize");

	/* clean up object here */
	G_OBJECT_CLASS (gst_spinnaker_src_parent_class)->finalize (object);
}

//queries camera devices and begins acquisition
static gboolean
gst_spinnaker_src_start (GstBaseSrc * bsrc)
{
	GstSpinnakerSrc *src = GST_SPINNAKER_SRC (bsrc);
    size_t numCameras = 0;

	GST_DEBUG_OBJECT (src, "start");
	
  	spinError errReturn = SPINNAKER_ERR_SUCCESS;
  	spinError err = SPINNAKER_ERR_SUCCESS;

	//grab system reference
    EXEANDCHECK(spinSystemGetInstance(&src->hSystem));

	//query available cameras
    EXEANDCHECK(spinCameraListCreateEmpty(&src->hCameraList));
	GST_DEBUG_OBJECT (src, "getting camera list");
    EXEANDCHECK(spinSystemGetCameras(src->hSystem, src->hCameraList));
	GST_DEBUG_OBJECT (src, "getting number of cameras");
    EXEANDCHECK(spinCameraListGetSize(src->hCameraList, &numCameras));
	
	// display error when no camera has been found
	if (numCameras==0){
		GST_ERROR_OBJECT(src, "No Flycapture device found.");
		
   	 // Clear and destroy camera list before releasing system
    	EXEANDCHECK(spinCameraListClear(src->hCameraList));

    	EXEANDCHECK(spinCameraListDestroy(src->hCameraList));

    	// Release system
    	EXEANDCHECK(spinSystemReleaseInstance(src->hSystem));

		GST_ERROR_OBJECT(src, "No device found.");
		goto fail;
	}

    // Select camera
  	spinCamera hCamera = NULL;
	GST_DEBUG_OBJECT (src, "selecting camera");
    EXEANDCHECK(spinCameraListGet(src->hCameraList, src->cameraID, &hCamera));
	GST_DEBUG_OBJECT (src, "initializing camera");
    EXEANDCHECK(spinCameraInit(hCamera));
    
	// Retrieve GenICam nodemap
    spinNodeMapHandle hNodeMap = NULL;

    err = spinCameraGetNodeMap(hCamera, &hNodeMap);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to retrieve GenICam nodemap. Aborting with error %d...\n\n", err);
        return err;
    }
	    err = ConfigureCustomImageSettings(hNodeMap);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        return err;
    }

	//starts camera acquisition. Doesn't actually fill the gstreamer buffer. see create function
	GST_DEBUG_OBJECT (src, "starting acquisition");
    EXEANDCHECK(spinCameraBeginAcquisition(hCamera));
	EXEANDCHECK(spinCameraRelease(hCamera));
	// NOTE:
	// from now on, the "deviceContext" handle can be used to access the camera board.
	// use fc2DestroyContext to end the usage
	src->cameraPresent = TRUE;

	return TRUE;

	fail:

    // Clear and destroy camera list before releasing system
    spinCameraListClear(src->hCameraList);

    spinCameraListDestroy(src->hCameraList);

    // Release system
    spinSystemReleaseInstance(src->hSystem);

	return FALSE;
}

//stops streaming and closes the camera
static gboolean
gst_spinnaker_src_stop (GstBaseSrc * bsrc)
{
	GstSpinnakerSrc *src = GST_SPINNAKER_SRC (bsrc);

	GST_DEBUG_OBJECT (src, "stop");
	spinImage hCamera = NULL;
	EXEANDCHECK(spinCameraListGet(src->hCameraList, src->cameraID, &hCamera));
  	EXEANDCHECK(spinCameraEndAcquisition(hCamera));
  	EXEANDCHECK(spinCameraDeInit(hCamera));
  	EXEANDCHECK(spinCameraRelease(hCamera));

	EXEANDCHECK(spinCameraListClear(src->hCameraList));
	EXEANDCHECK(spinCameraListDestroy(src->hCameraList));
	EXEANDCHECK(spinSystemReleaseInstance(src->hSystem));

	gst_spinnaker_src_reset (src);
	GST_DEBUG_OBJECT (src, "stop completed");
	return TRUE;

	fail:  
	return TRUE;
}

static GstCaps *
gst_spinnaker_src_get_caps (GstBaseSrc * bsrc, GstCaps * filter)
{
	GstSpinnakerSrc *src = GST_SPINNAKER_SRC (bsrc);
	GstCaps *caps;

	GstVideoInfo vinfo;

	gst_video_info_init(&vinfo);

	vinfo.width = src->nWidth;
	vinfo.height = src->nHeight;
	vinfo.fps_n = 0; //0 means variable FPS
	vinfo.fps_d = 1;
	vinfo.interlace_mode = GST_VIDEO_INTERLACE_MODE_PROGRESSIVE;
	vinfo.finfo = gst_video_format_get_info(DEFAULT_GST_VIDEO_FORMAT);
  
	caps = gst_video_info_to_caps(&vinfo);

	GST_DEBUG_OBJECT (src, "The caps are %" GST_PTR_FORMAT, caps);

	return caps;
}

static gboolean
gst_spinnaker_src_set_caps (GstBaseSrc * bsrc, GstCaps * caps)
{
	GstSpinnakerSrc *src = GST_SPINNAKER_SRC (bsrc);
	GstVideoInfo vinfo;

	//Currently using fixed caps
	GST_DEBUG_OBJECT (src, "The caps being set are %" GST_PTR_FORMAT, caps);
	src->acq_started = TRUE;

	return TRUE;

	unsupported_caps:
	GST_ERROR_OBJECT (src, "Unsupported caps: %" GST_PTR_FORMAT, caps);
	return FALSE;

	fail:
	return FALSE;
}

//Grabs next image from camera and puts it into a gstreamer buffer
#ifdef OVERRIDE_CREATE
static GstFlowReturn
gst_spinnaker_src_create (GstPushSrc * psrc, GstBuffer ** buf)
{
	spinError err = SPINNAKER_ERR_SUCCESS;
	GstSpinnakerSrc *src = GST_SPINNAKER_SRC (psrc);
	GstMapInfo minfo;

	//query camera and grab next image
	spinImage hResultImage = NULL;
	spinImage hCamera = NULL;
	EXEANDCHECK(spinCameraListGet(src->hCameraList, src->cameraID, &hCamera));
	EXEANDCHECK(spinCameraGetNextImage(hCamera, &hResultImage));
	EXEANDCHECK(spinCameraRelease(hCamera));

	bool8_t isIncomplete = False;
	bool8_t hasFailed = False;

	//check if image is complete 
	//WARNING: This returns a boolean and is not handled if the image is incomplete
	EXEANDCHECK(spinImageIsIncomplete(hResultImage, &isIncomplete));

	// Create a new buffer for the image
	//printf("Bytes per pixel are %d\n"),src->nBytesPerPixel;
	//*buf = gst_buffer_new_and_alloc (src->nHeight * src->nWidth * src->nBytesPerPixel);
	*buf = gst_buffer_new_and_alloc (src->nHeight * src->nWidth * 16);
	//printf("Bytes per pixel are %d\n"),src->nBytesPerPixel;

	gst_buffer_map (*buf, &minfo, GST_MAP_WRITE);


	spinImage hConvertedImage = NULL;

    err = spinImageCreateEmpty(&hConvertedImage);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to create image. Non-fatal error %d...\n\n", err);
        hasFailed = True;
    }

    err = spinImageConvert(hResultImage, PixelFormat_Mono8, hConvertedImage);
    if (err != SPINNAKER_ERR_SUCCESS)
    {
        printf("Unable to convert image. Non-fatal error %d...\n\n", err);
        hasFailed = True;
    }


	size_t imageSize;
	EXEANDCHECK(spinImageGetBufferSize(hConvertedImage, &imageSize));
	//printf("Converted Image size is %d\n",imageSize);

	EXEANDCHECK(spinImageGetBufferSize(hResultImage, &imageSize));
	//printf("Source Image size is %d\n",imageSize);

	//printf("nPitch size is %d\n",src->nPitch);
	//printf("gst_stride size is %d\n",src->gst_stride);

	//grab pointer to image data	
	void *data;
	EXEANDCHECK(spinImageGetData(hConvertedImage, &data)); 

	//copy image data into gstreamer buffer
	for (int i = 0; i < src->nHeight; i++) {
		//printf("Copy line %d \n",i);
		memcpy (minfo.data + i * src->gst_stride, data + i * src->nPitch, src->nPitch);
	}

	//release image and buffer
	EXEANDCHECK(spinImageRelease(hResultImage));
	//EXEANDCHECK(spinImageRelease(hConvertedImage));
	gst_buffer_unmap (*buf, &minfo);

    src->duration = 1000000000.0/src->framerate; 
	// If we do not use gst_base_src_set_do_timestamp() we need to add timestamps manually
	src->last_frame_time += src->duration;   // Get the timestamp for this frame
	if(!gst_base_src_get_do_timestamp(GST_BASE_SRC(psrc))){
		GST_BUFFER_PTS(*buf) = src->last_frame_time;  // convert ms to ns
		GST_BUFFER_DTS(*buf) = src->last_frame_time;  // convert ms to ns
	}
	GST_BUFFER_DURATION(*buf) = src->duration;
	GST_DEBUG_OBJECT(src, "pts, dts: %" GST_TIME_FORMAT ", duration: %d ms", GST_TIME_ARGS (src->last_frame_time), GST_TIME_AS_MSECONDS(src->duration));

	// count frames, and send EOS when required frame number is reached
	GST_BUFFER_OFFSET(*buf) = src->n_frames;  // from videotestsrc
	src->n_frames++;
	GST_BUFFER_OFFSET_END(*buf) = src->n_frames;  // from videotestsrc
	if (psrc->parent.num_buffers>0)  // If we were asked for a specific number of buffers, stop when complete
		if (G_UNLIKELY(src->n_frames >= psrc->parent.num_buffers))
			return GST_FLOW_EOS;

	return GST_FLOW_OK;
	fail:
	return GST_FLOW_ERROR;
}
#endif // OVERRIDE_CREATE

static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  return gst_element_register (plugin, "spinnakersrc", GST_RANK_NONE,
      GST_TYPE_SPINNAKER_SRC);

}
/* FIXME: these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
   
#ifndef VERSION
#define VERSION "0.0.1"
#endif
#ifndef PACKAGE
#define PACKAGE "GST Spinnaker"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "spinnakersrc"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "github.com/thompd27"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    spinnaker,
    "plugin for interfacing with FLIR machine vision cameras based on the spinnaker API",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN);
