//-------------------------------------------------------------
/**
\file      GxSingleCamMono.cpp
\brief     Sample to show how to acquire mono image continuously and save ppm images
\version   1.1.2312.9041
\date      2012.12.24
*/
//-------------------------------------------------------------

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50              ///< Save image file name length

#define PIXFMT_CVT_FAIL         -1              ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS      0               ///< PixelFormatConvert success

//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        return emStatus;                   \
    }

GX_DEV_HANDLE g_hDevice = NULL;              ///< Device handle
bool g_bAcquisitionFlag = false;             ///< Thread running flag
bool g_bSavePPMImage = false;                ///< Save raw image flag
pthread_t g_nAcquisitonThreadID = 0;         ///< Thread ID of Acquisition thread

unsigned char* g_pMonoImageBuf = NULL;       ///< Memory for Mono image

uint32_t g_nPayloadSize = 0;                  ///< Payload size

//Allocate the memory for pixel format transform 
void PreForAcquisition();

//Release the memory allocated
void UnPreForAcquisition();

//Convert frame date to suitable pixel format
int PixelFormatConvert(PGX_FRAME_BUFFER);

//Save one frame to PPM image file
void SavePPMFile(uint32_t, uint32_t);

//Acquisition thread function
void *ProcGetImage(void*);

//Get description of error
void GetErrorString(GX_STATUS);

int main()
{
    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("Sample to show how to acquire mono image continuously and save ppm image.\n");
    printf("version: 1.1.2312.9041\n");
    printf("-------------------------------------------------------------\n");
    printf("\n");

    printf("Initializing......"); 
    printf("\n\n");
     
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    uint32_t ui32DeviceNum = 0;

    //Initialize libary
    emStatus = GXInitLib(); 
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return 0;
    }

    //Get device enumerated number
    emStatus = GXUpdateAllDeviceList(&ui32DeviceNum, 1000);
    if(emStatus != GX_STATUS_SUCCESS)
    { 
        GetErrorString(emStatus);
        GXCloseLib();
        return 0;
    }

    //If no device found, app exit
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return 0;
    }

    //Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return 0;           
    }

    //Get Device Info
    printf("***********************************************\n");
    //Get libary version
    printf("<Libary Version : %s>\n", GXGetLibVersion());

	//Get string length of Vendor name
	GX_STRING_VALUE stStrVendorName;
	emStatus = GXGetStringValue(g_hDevice, "DeviceVendorName", &stStrVendorName);
	GX_VERIFY_EXIT(emStatus);
	printf("<Vendor Name : %s>\n", stStrVendorName.strCurValue);

	//Get string length of Model name
	GX_STRING_VALUE stStrModelName;
	emStatus = GXGetStringValue(g_hDevice, "DeviceModelName", &stStrModelName);
	GX_VERIFY_EXIT(emStatus);
	printf("<Model Name : %s>\n", stStrModelName.strCurValue);

	//Get string length of Serial number
	GX_STRING_VALUE stStrSerialNumber;
	emStatus = GXGetStringValue(g_hDevice, "DeviceSerialNumber", &stStrSerialNumber);
	GX_VERIFY_EXIT(emStatus);
	printf("<Serial Number : %s>\n", stStrSerialNumber.strCurValue);

	//Get string length of Device version
	GX_STRING_VALUE stStrDeviceVersion;
	emStatus = GXGetStringValue(g_hDevice, "DeviceVersion", &stStrDeviceVersion);
	GX_VERIFY_EXIT(emStatus);
	printf("<Device Version : %s>\n", stStrDeviceVersion.strCurValue);
    printf("***********************************************\n");


    //Get the type of Bayer conversion. whether is a color camera.
	GX_NODE_ACCESS_MODE emAccessMode;
	bool bColorFilter = false;
	emStatus = GXGetNodeAccessMode(g_hDevice, "PixelColorFilter", &emAccessMode);
	GX_VERIFY_EXIT(emStatus);
	bColorFilter = ((emAccessMode == GX_NODE_ACCESS_MODE_WO) || (emAccessMode == GX_NODE_ACCESS_MODE_RO) || (emAccessMode == GX_NODE_ACCESS_MODE_RW)) ? true : false;

	uint32_t nDSNum = 0;
	emStatus = GXGetDataStreamNumFromDev(g_hDevice, &nDSNum);
	GX_VERIFY_EXIT(emStatus);

	if (nDSNum < 1)
	{
		printf("Failed to obtain the number of streams\n");
		return GX_STATUS_ERROR;
	}

	GX_DS_HANDLE phDS = NULL;
	emStatus = GXGetDataStreamHandleFromDev(g_hDevice, 1, &phDS);
	GX_VERIFY_EXIT(emStatus);

	emStatus = GXGetPayLoadSize(phDS, &g_nPayloadSize);
	GX_VERIFY_EXIT(emStatus);

    //This app only support mono cameras
    if (bColorFilter)
    {
        printf("<This app only support mono cameras! App Exit!>\n");
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        return 0;
    }

    printf("\n");
    printf("Press [a] or [A] and then press [Enter] to start acquisition\n");
    printf("Press [s] or [S] and then press [Enter] to save one ppm image\n");
    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("\n");

    int nStartKey = 0;
    bool bWaitStart = true;
    while (bWaitStart)
    {
        nStartKey = getchar();
        switch(nStartKey)
        {
            //press 'a' and [Enter] to start acquisition
            //press 'x' and [Enter] to exit
            case 'a':
            case 'A':
                //Start to acquisition
                bWaitStart = false;
                break;
            case 'S':
            case 's':
                printf("<Please start acquisiton before saving image!>\n");
                break;
            case 'x':
            case 'X':
                //App exit
                GXCloseDevice(g_hDevice);
                g_hDevice = NULL;
                GXCloseLib();
                printf("<App exit!>\n");
                return 0;
            default:
                break;
        }
    }

	//Set acquisition mode
	emStatus = GXSetEnumValueByString(g_hDevice, "AcquisitionMode", "Continuous");
	GX_VERIFY_EXIT(emStatus);

	//Set trigger mode
	emStatus = GXSetEnumValueByString(g_hDevice, "TriggerMode", "Off");
	GX_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
    GX_VERIFY_EXIT(emStatus);

	GX_NODE_ACCESS_MODE emStreamTransferSizeIm;
	emStatus = GXGetNodeAccessMode(g_hDevice, "StreamTransferSize", &emStreamTransferSizeIm);
	GX_VERIFY_EXIT(emStatus);
	bool bStreamTransferSize = ((emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_RO) || (emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_WO) || (emStreamTransferSizeIm == GX_NODE_ACCESS_MODE_RW)) ? true : false;
    if(bStreamTransferSize)
    {
        //Set size of data transfer block
		emStatus = GXSetIntValue(g_hDevice, "StreamTransferSize", ACQ_TRANSFER_SIZE);
		GX_VERIFY_EXIT(emStatus);
    }

	GX_NODE_ACCESS_MODE emStreamTransferUrbIm;
	emStatus = GXGetNodeAccessMode(g_hDevice, "StreamTransferNumberUrb", &emStreamTransferUrbIm);
	GX_VERIFY_EXIT(emStatus);
	bool bStreamTransferNumberUrb = ((emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_RO) || (emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_WO) || (emStreamTransferUrbIm == GX_NODE_ACCESS_MODE_RW)) ? true : false;
    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
		emStatus = GXSetIntValue(g_hDevice, "StreamTransferNumberUrb", ACQ_TRANSFER_NUMBER_URB);
		GX_VERIFY_EXIT(emStatus);
    }

    //Allocate the memory for pixel format transform 
    PreForAcquisition();

    //Device start acquisition
    emStatus = GXStreamOn(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    //Start acquisition thread, if thread create failed, exit this app
    int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, NULL);
    if(nRet != 0)
    {
        //Release the memory allocated
        UnPreForAcquisition();

        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();

        printf("<Failed to create the acquisition thread, App Exit!>\n");
        exit(nRet);
    }

    //Main loop
    bool bRun = true;
    while(bRun == true)
    {
        int nKey = getchar();
        //press 's' and [Enter] for save image
        //press 'x' and [Enter] for exit
        switch(nKey)
        {
            //Save PPM Image
            case 'S':
            case 's':
                g_bSavePPMImage = true;
                break;
            //Exit app
            case 'X': 
            case 'x':
                bRun = false;
                break;
            default:
                break;
        }   
    }

    //Stop Acquisition thread
    g_bAcquisitionFlag = false;
    pthread_join(g_nAcquisitonThreadID, NULL);

    //Device stop acquisition
    emStatus = GXStreamOff(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    //Release the resources and stop acquisition thread
    UnPreForAcquisition();

    //Close device
    emStatus = GXCloseDevice(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    //Release libary
    emStatus = GXCloseLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }

    printf("<App exit!>\n");
    return 0;
}

//-------------------------------------------------
/**
\brief Save PPM image
\param ui32Width[in]       image width
\param ui32Height[in]      image height
\return void
*/
//-------------------------------------------------
void SavePPMFile(uint32_t ui32Width, uint32_t ui32Height)
{
    char szName[FILE_NAME_LEN] = {0};

    static int nRawFileIndex = 0;

    if(g_pMonoImageBuf != NULL)
    {
        FILE* phImageFile = NULL;
        snprintf(szName, FILE_NAME_LEN, "Frame_%d.ppm", nRawFileIndex++);
        phImageFile = fopen(szName,"wb");
        if (phImageFile == NULL)
        {
            printf("Create or Open %s failed!\n", szName);
            return;
        }
        fprintf(phImageFile, "P5\n%u %u 255\n", ui32Width, ui32Height);
        fwrite(g_pMonoImageBuf, 1, g_nPayloadSize, phImageFile);
        fclose(phImageFile);
        phImageFile = NULL;
        printf("Save %s succeed\n", szName);
    }
    else
    {
        printf("Save %s failed!\n", szName);
    }
    return;
}

//-------------------------------------------------
/**
\brief Convert frame date to suitable pixel format
\param pParam[in]           pFrameBuffer       FrameData from camera
\return void
*/
//-------------------------------------------------
int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer)
{
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_MONO8:
        {
            memcpy(g_pMonoImageBuf, pFrameBuffer->pImgBuf, g_nPayloadSize);
            break;
        }
        case GX_PIXEL_FORMAT_MONO10:
        case GX_PIXEL_FORMAT_MONO12:
        {
            //Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, g_pMonoImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
    }
    return PIXFMT_CVT_SUCCESS;
}

//-------------------------------------------------
/**
\brief Start acquisition thread
\return void
*/
//-------------------------------------------------
void PreForAcquisition()
{
    g_pMonoImageBuf = new unsigned char[g_nPayloadSize];

    return;
}

//-------------------------------------------------
/**
\brief Release the resources and stop acquisition thread
\return void
*/
//-------------------------------------------------
void UnPreForAcquisition()
{   
    //Release resources
    if (g_pMonoImageBuf != NULL)
    {
        delete[] g_pMonoImageBuf;
        g_pMonoImageBuf = NULL;
    }

    return;
}

//-------------------------------------------------
/**
\brief Acquisition thread function
\param pParam       thread param, not used in this app
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void* pParam)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;

    while(g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue
        emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                GetErrorString(emStatus);
                break;
            }
        }

        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
        else
        {
            ui32FrameCount++;
            time (&lEnd);
            // Print acquisition info each second.
            if (lEnd - lInit >= 1)
            {   
                printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %lu>\n", 
                    ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
                ui32FrameCount = 0;
            }

            if (g_bSavePPMImage)
            {   
                int nRet = PixelFormatConvert(pFrameBuffer);
                if (nRet == PIXFMT_CVT_SUCCESS)
                {
                    SavePPMFile(pFrameBuffer->nWidth, pFrameBuffer->nHeight);
                }
                else
                {
                    printf("PixelFormat Convert failed!\n");
                }
                g_bSavePPMImage = false;
            }
        }

        emStatus = GXQBuf(g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }  
    }
    printf("<Acquisition thread Exit!>\n");

    return 0;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    
    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }
    
    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
    
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

