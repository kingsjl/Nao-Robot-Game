

#include <iostream>
#include "genericvideomodule.h"

// For resolution and colorspace definitions + utility functions.
#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>

#include "opencv/highgui.h"

#include <qi/log.hpp>
#define TEST 1

using namespace std;

const int hsv_shift = 12;
int lookuptable[256][256][256];
static const int div_table[] = {
	0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
	130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
	65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
	43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
	32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
	26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
	21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
	18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
	16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
	14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
	13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
	11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
	10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
	10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
	9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
	8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
	8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
	7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
	7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
	6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
	6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
	6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
	5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
	5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
	5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
	5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
	5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
	4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
	4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
	4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
	4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
	4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
};
const uchar icvSaturate8u_cv[] = 
{
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,
	16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,
	32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
	48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
	64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
	80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,
	96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
	144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
	208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255
};
#define CV_FAST_CAST_8U(t)  (assert(-256 <= (t) || (t) <= 512), icvSaturate8u_cv[(t)+256])
#define CV_CALC_MIN_8U(a,b) (a)-= CV_FAST_CAST_8U((a) - (b))
#define CV_CALC_MAX_8U(a,b) (a)+= CV_FAST_CAST_8U((b) - (a))

//从opencv中抠出来的将rgb转为hsv的代码



namespace AL
{
GenericVideoModule::GenericVideoModule( boost::shared_ptr<ALBroker> pBroker, const std::string& pName ):
  ALModule(pBroker , pName),
  fRegisteredToVideoDevice(false),
  fIplImageHeader(NULL)
{

  // Describe the module here.
  setModuleDescription( "This is an example of a generic video module. "
    "It can be used to save images returned by the camera. "
    "It internally uses OpenCv's cvSaveImage function. "
    "Refer to the Example codes section below for an example of use.");

  // Define the module example.
  addModuleExample( "Python",
                     "  # Create a proxy to the module \n"
                     "  sampleProxy = ALProxy('GenericVideoModule', '127.0.0.1', 9559)\n\n"
                     "  # Register our module to the Video Input Module. \n"
                     "  sampleProxy.registerToVideoDevice(1, 13)\n\n"
                     "  # Save image in remote mode. \n"
                     "  sampleProxy.saveImageRemote('/home/nao/img', 'jpg') \n\n"
                     "  # Unregister.\n"
                     "  sampleProxy.unRegisterFromVideoDevice()\n"
                  );

  // Define bound methods with their description.
  functionName( "registerToVideoDevice", getName(), "Register to the V.I.M." );
  addParam("resolution", "Resolution requested.");
  addParam("colorSpace", "Colorspace requested.");
  BIND_METHOD( GenericVideoModule::registerToVideoDevice );

  functionName( "unRegisterFromVideoDevice", getName(), "Unregister from the V.I.M." );
  BIND_METHOD( GenericVideoModule::unRegisterFromVideoDevice );

  functionName( "saveImageLocal", getName(), "Save an image received from the camera." );
  addParam( "name", "name and path of the picture (without format extension)" );
  addParam( "imageFormat", "format extension such as bmp, jpg, tiff, etc.)" );
  BIND_METHOD( GenericVideoModule::saveImageLocal );

  functionName( "saveImageRemote", getName(), "Save an image received from the camera. to be used if the genericvideomodule is a remote module." );
  addParam( "name", "name and path of the picture (without format extension)" );
  addParam( "imageFormat", "format extension such as bmp, jpg, tiff, etc.)" );
  BIND_METHOD( GenericVideoModule::saveImageRemote );

functionName("initGaoshuai", getName(), "Init the LookupTable before you call the function of Gaoshuai");
  addParam("H_min", "The maximum value of H");
  addParam("H_max", "The minimum value of H");
  addParam("S_min", "The maximum value of S");
  addParam("S_max", "The minimum value of S");
  addParam("V_min", "The maximum value of V");
  addParam("V_max", "The minimum value of V");
  BIND_METHOD(GenericVideoModule::initGaoshuai);

functionName("Gaoshuai", getName(), "The find the obstacle with HSV");
  BIND_METHOD(GenericVideoModule::Gaoshuai);

}


void GenericVideoModule::exit()
{
  AL::ALModule::exit();
}


void GenericVideoModule::init() {

  // Create a proxy to the ALVideoDevice.
  try {
    fCamProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(getParentBroker()));
  } catch (const AL::ALError& e) {
    qiLogError("vision.genericvideomodule") << "Error while getting proxy on ALVideoDevice.  Error msg " << e.toString() << std::endl;
    GenericVideoModule::exit();
    return;
  }
  if(fCamProxy == NULL)
  {
    qiLogError("vision.genericvideomodule") << "Error while getting proxy on ALVideoDevice. Check ALVideoDevice is running." << std::endl;
    GenericVideoModule::exit();
    return;
  }

  qiLogInfo("vision.genericvideomodule") << "Use registerToVideoDevice + "
    "saveImageLocal + unRegisterFromVideoDevice to save images." << std::endl;

  memProxy = boost::shared_ptr<ALMemoryProxy>(new ALMemoryProxy(getParentBroker()));
  memProxy->insertData("FindObstacle/cx",-1);
  memProxy->insertData("FindObstacle/cy",-1);
  FOCentreX = (int*)memProxy->getDataPtr("FindObstacle/cx");
  FOCentreY = (int*)memProxy->getDataPtr("FindObstacle/cy");
}


GenericVideoModule::~GenericVideoModule() {
  // Release Image Header if client code forgot to call unregisterFromVim.
  if (fIplImageHeader)
    cvReleaseImageHeader(&fIplImageHeader);

  // Unregister the video module.
  try
  {
    if(fCamProxy)
      fCamProxy->unsubscribe(fVideoClientName);

    fCamProxy.reset();
  }
  catch(const AL::ALError& e)
  {
    qiLogError("vision.genericvideomodule") <<  e.toString() << std::endl;
  }
}

/**
 * registerToVIM
 */
void GenericVideoModule::registerToVideoDevice(const int &pResolution, const int &pColorSpace) {

  // If we've already registered a module, we need to unregister it first !
  if (fRegisteredToVideoDevice) {
    throw ALError(getName(), "registerToVideoDevice()", "A video module has already been "
      "registered. Call unRegisterFromVideoDevice() before trying to register a new module.");
  }

  // GVM Name that we're going to use to register.
  const std::string kOriginalName = "genericvideomodule";
  int imgWidth = 0;
  int imgHeight = 0;
  int imgNbLayers = 0;
  const int kImgDepth = 8;
  const int kFps = 5;

  // Release Image Header if it has been allocated before.
  if (fIplImageHeader)
    cvReleaseImageHeader(&fIplImageHeader);

  setSizeFromResolution(pResolution, imgWidth, imgHeight);
  imgNbLayers = getNumLayersInColorSpace(pColorSpace);

  if (imgWidth == -1 || imgWidth == -1 || imgNbLayers == -1) {
    throw ALError(getName(), "registerToVideoDevice()", "Invalid resolution or color space.");
  }

  // Allocate our Image header.
  fIplImageHeader = cvCreateImageHeader(cvSize(imgWidth, imgHeight), kImgDepth,
    imgNbLayers);

  if (!fIplImageHeader) {
    throw ALError(getName(), "registerToVideoDevice()", "Fail to allocate OpenCv image header.");
  }

  // Call the "subscribe" function with the given parameters.
  if(fCamProxy)
    fVideoClientName = fCamProxy->subscribe(kOriginalName, pResolution, pColorSpace, kFps );

  qiLogInfo("vision.genericvideomodule") << "Module registered as " << fVideoClientName << std::endl;

  // Registration is successful, set fRegisteredToVim to true.
  fRegisteredToVideoDevice = true;
}

void GenericVideoModule::initGaoshuai(const int &H_min,const int &H_max,const int &S_min,const int &S_max,const int &V_min,const int &V_max)
{
	int r,g,b;


	for (r=0; r < 256; r++)
	{
		for (g=0; g < 256; g++)
		{
			for (b=0; b < 256; b++)
			{
				int h,s,v;
	
				v = b;
				int vmin = b, diff;
				int vr, vg;

				CV_CALC_MAX_8U( v, g );
				CV_CALC_MAX_8U( v, r );
				CV_CALC_MIN_8U( vmin, g );
				CV_CALC_MIN_8U( vmin, r );

				diff = v - vmin;
				vr = v == r ? -1 : 0;
				vg = v == g ? -1 : 0;

				s = diff * div_table[v] >> hsv_shift;
				h = (vr & (g - b)) +
					(~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
				h = ((h * div_table[diff] * 15 + (1 << (hsv_shift + 6))) >> (7 + hsv_shift))\
					+ (h < 0 ? 30*6 : 0);
				lookuptable[b][g][r] = -1;
					if (  h >= H_min && h <= H_max
						&&s >= S_min && s <= S_max
						&&v >= V_min && v <= V_max)
					{ 
						lookuptable[b][g][r] = 0;
						break;
					}
				
			}
		}
	}
}

void GenericVideoModule::Gaoshuai( )
{
	// Check that a video module has been registered.
	if (!fRegisteredToVideoDevice) 
	{
    		throw ALError(getName(), "FindBall()",  "No video module is currently "
      		"registered! Call registerToVideoDevice() first.");
  	}

	#ifdef GENERICVIDEOMODULE_IS_REMOTE
  	// If this module is running in remote mode, we shouldn't use saveImageLocal.
  	throw ALError(getName(), "FindBall()", "Module is run in remote mode, ");
	#else
 	//ALValue results;
        //results = (fCamProxy->getImageRemote(fVideoClientName ));
	ALImage* imageIn = NULL;

  	// Now you can get the pointer to the video structure.
  	imageIn = (ALImage*)fCamProxy->getImageLocal(fVideoClientName);
  	if (!imageIn) 
	{
    		throw ALError(getName(), "saveImageLocal", "Invalid image returned.");
  	}
  const long long timeStamp = imageIn->getTimeStamp();
  const int seconds = (int)(timeStamp/1000000LL);

  	fIplImageHeader->imageData = (char*)imageIn->getData();

	IplImage* src = cvCreateImage(cvGetSize(fIplImageHeader),8,3);
	cvCopyImage(fIplImageHeader, src);

	
	IplImage* hsv = cvCreateImage( cvGetSize(src), 8, 3 );
	IplImage* dst_gray = cvCreateImage( cvGetSize(src), 8, 1 );

	int w,h;
	cvCvtColor( src, hsv, CV_BGR2HSV );//颜色空间转换函数

	cvZero( dst_gray );//让矩阵的值都为零
	///////////////////////////////////////////////////////////////////////////////////////////
	//以下开始实时处理
	//获得颜色图
	int r,g,b;
	for (h=0; h < src->height; h++)
	{
		for (w=0; w < src->width; w++)
		{
			uchar* temp_ptr = &((uchar*)(src->imageData + src->widthStep*h))[w*3];
			b = temp_ptr[0] ;
			g = temp_ptr[1] ;
			r = temp_ptr[2] ;
			int color = lookuptable[b][g][r];
			if (color > -1)
			{
				dst_gray->imageData[dst_gray->widthStep*h+w] = 255;
			}
		}
	}
	cvErode(dst_gray,dst_gray,0,3);
	cvDilate(dst_gray,dst_gray,0,3);
	cvSmooth(dst_gray,dst_gray,CV_MEDIAN,9,9);

	CvMemStorage *storagetemp = NULL;

	if (storagetemp == NULL)
	{
		storagetemp = cvCreateMemStorage(0);
	}
	else
		cvClearMemStorage(storagetemp);
	CvSeq* contour = 0;
	float area = 0;

	cvFindContours(dst_gray,storagetemp,&contour);

	CvSeq* maxContour = 0;
	float maxArea = 0;
	for (;contour!=0;contour = contour->h_next)
	{
		area = fabs(cvContourArea(contour,CV_WHOLE_SEQ));
		if (area >= maxArea)
		{
			maxArea = area;
			maxContour = contour;
		}
	}
	int cx,cy;
	if (maxArea != 0)
	{
		CvRect arect = cvBoundingRect( maxContour, 0 ); 
		cvRectangle(src,cvPoint(arect.x,arect.y),cvPoint(arect.x+arect.width,arect.y+arect.height),CV_RGB(255,0,0),2);		

		*FOCentreX = arect.x+arect.width/2;
		*FOCentreY = arect.y+arect.height/2;
		cx = arect.x+arect.width/2;
		cy = arect.y+arect.height/2;
		
	}
	else 
	{
		*FOCentreX = -1;
		*FOCentreY = -1;
		cx = -1;
		cy = -1;

	}

 	std::stringstream ss;
  	ss << "/home/nao/gaoshuai/image/" << seconds << "_" << cx << "_" << cy << ".jpg" ;
  	const std::string kImageNameFull = ss.str();

  	try {
    	cvSaveImage(kImageNameFull.c_str(), src);
    	qiLogInfo("vision.genericvideomodule") << "Image saved as " << kImageNameFull << std::endl;
  	}
  	catch(const cv::Exception& e) {
    	qiLogError("vision.genericvideomodule") << "OpenCV can't save the image "
                                                << "with this format. (e.g. "
                                                << "incompatible file format"
                                                << " / no space left)  Error msg "
                                                << e.err << std::endl;
  	}
//=======================================================================================

	cvReleaseImage(&src);		
	cvReleaseImage(&hsv);		
	//cvReleaseImage(&hsi);
	cvReleaseImage(&dst_gray);
//	cvReleaseImage(&draw);
	
	cvReleaseMemStorage(&storagetemp);


  	fCamProxy->releaseImage(fVideoClientName);

	#endif
}

/**
 * unRegisterFromVIM
 */
void GenericVideoModule::unRegisterFromVideoDevice() {

  if (!fRegisteredToVideoDevice) {
    throw ALError(getName(), "unRegisterFromVideoDevice()", "No video module is currently "
      "registered! Call registerToVideoDevice first.");
  }

  // Release Image Header if it has been allocated.
  if (fIplImageHeader)
    cvReleaseImageHeader(&fIplImageHeader);

  qiLogInfo("vision.genericvideomodule") << "try to unregister " << fVideoClientName << " module." << std::endl;
  if(fCamProxy)
    fCamProxy->unsubscribe(fVideoClientName);

  qiLogInfo("vision.genericvideomodule") << "Done." << std::endl;

  // UnRegistration is successful, set fRegisteredToVim to false.
  fRegisteredToVideoDevice = false;
}


/**
 * saveImage : save the last image received.
 * @param pName name of the file
 */
void GenericVideoModule::saveImageLocal(const std::string& pName, const std::string& pImageFormat) {

  // Check that a video module has been registered.
  if (!fRegisteredToVideoDevice) {
    throw ALError(getName(), "saveImageLocal()",  "No video module is currently "
      "registered! Call registerToVideoDevice() first.");
  }

#ifdef GENERICVIDEOMODULE_IS_REMOTE
  // If this module is running in remote mode, we shouldn't use saveImageLocal.
  throw ALError(getName(), "saveImageLocal()", "Module is run in remote mode, "
    "use saveImageRemote instead !");
#else

  ALImage* imageIn = NULL;

  // Now you can get the pointer to the video structure.
  imageIn = (ALImage*)fCamProxy->getImageLocal(fVideoClientName);
  if (!imageIn) {
    throw ALError(getName(), "saveImageLocal", "Invalid image returned.");
  }


  // You can get some image information that you may find useful.
  //const int width = imageIn->fWidth;
  //const int height = imageIn->fHeight;
  //const int nbLayers = imageIn->fNbLayers;
  //const int colorSpace = imageIn->fColorSpace;
  const long long timeStamp = imageIn->getTimeStamp();
  const int seconds = (int)(timeStamp/1000000LL);

  // Set the buffer we received to our IplImage header.
  fIplImageHeader->imageData = (char*)imageIn->getData();

  xSaveIplImage(fIplImageHeader, pName, pImageFormat, seconds);

  // Now that you're done with the (local) image, you have to release it from the V.I.M.
  fCamProxy->releaseImage(fVideoClientName);

#endif
}


/**
 * saveImageRemote : save the last image received. To be used if genericvideomodule is a remote module.
 * @param pName name of the file
 */
void GenericVideoModule::saveImageRemote(const std::string& pName, const std::string& pImageFormat ) {

  // Check that a video module has been registered.
  if (!fRegisteredToVideoDevice) {
    throw ALError(getName(), "saveImageRemote()",  "No video module is currently "
      "registered! Call registerToVideoDevice() first.");
  }

  ALValue results;

  results = (fCamProxy->getImageRemote(fVideoClientName ));

  if (results.getType()!= ALValue::TypeArray && results.getSize() != 7) {
    throw ALError(getName(), "saveImageRemote", "Invalid image returned.");
  }

  //const int size = results[6].getSize();

  // You can get some image information that you may find useful.
  //const int width = (int) results[0];
  //const int height = (int) results[1];
  //const int nbLayers = (int) results[2];
  //const int colorSpace = (int) results[3];
  const long long timeStamp = ((long long)(int)results[4])*1000000LL + ((long long)(int)results[5]);
  const int seconds = (int)(timeStamp/1000000LL);

  // Set the buffer we received to our IplImage header.
  fIplImageHeader->imageData = (char*)(results[6].GetBinary());

  xSaveIplImage(fIplImageHeader, pName, pImageFormat, seconds);
}


// Actually perform the cvSaveImage operation.
void GenericVideoModule::xSaveIplImage(const IplImage* img,
                             const std::string& pName,
                             const std::string& pImageFormat,
                             int seconds) {

  std::stringstream ss;
  ss << pName << seconds << "." << pImageFormat;
  const std::string kImageNameFull = ss.str();

  try {
    cvSaveImage(kImageNameFull.c_str(), fIplImageHeader);
    qiLogInfo("vision.genericvideomodule") << "Image saved as " << kImageNameFull << std::endl;
  }
  catch(const cv::Exception& e) {
    qiLogError("vision.genericvideomodule") << "OpenCV can't save the image "
                                              << "with this format. (e.g. "
                                              << "incompatible file format"
                                              << " / no space left)  Error msg "
                                              << e.err << std::endl;
  }
}

} // namespace AL
