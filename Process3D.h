#pragma once

//文件读取
#include "itkImage.h"                   //数据类
#include "itkImageFileReader.h"         
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "itkRawImageIO.h"

#include <itkMultiThreaderBase.h>    //多线程
#include <itkMultiThreaderBase.h> 


//形态学处理
#include "itkFlatStructuringElement.h"         //创建特定形状大小的结构，球、十字、方形、环形、多边形
#include "itkBinaryBallStructuringElement.h"   //基本球形
#include "itkBinaryCrossStructuringElement.h"  ///交叉形
#include "itkBinaryMorphologicalOpeningImageFilter.h"//开运算
#include "itkBinaryMorphologicalClosingImageFilter.h"//闭运算
//#include <itkGrayscaleConnectedClosingImageFilter.h>  //灰度闭运算   报错
#include "itkGrayscaleErodeImageFilter.h"//灰度腐蚀
#include "itkGrayscaleDilateImageFilter.h"//灰度膨胀
#include "itkBinaryDilateImageFilter.h"//二值膨胀
#include "itkBinaryErodeImageFilter.h"//二值腐蚀
#include "itkBlackTopHatImageFilter.h"   //底帽
#include "itkWhiteTopHatImageFilter.h"   //顶帽


#include "itkBinaryThresholdImageFilter.h"  //二值化头文件 
#include "itkOtsuThresholdImageFilter.h"    //Otsu分割头文文件
#include "itkThresholdImageFilter.h"        //General Threshold 范围分割
#include "itkBinaryThresholdImageFilter.h"  //门限阈值分割


#include "itkMeanImageFilter.h"  //均值滤波

#include "itkMultiplyImageFilter.h"    //图像相乘
#include "itkSubtractImageFilter.h"    //减
#include "itkAndImageFilter.h"//与

//hessian分割
#include "itkHessian3DToVesselnessMeasureImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkHessianToObjectnessMeasureImageFilter.h"
#include "itkMultiScaleHessianBasedMeasureImageFilter.h"   //多尺度hessian滤波

#include "itkCastImageFilter.h"    //数据类型转换
#include "itkRescaleIntensityImageFilter.h"   //数值映射
#include "itkRegionOfInterestImageFilter.h"   //获取ROI区域

//连通域分析
#include "itkLabelImageToShapeLabelMapFilter.h"   
#include "itkConnectedComponentImageFilter.h"
#include "itkStatisticsLabelObject.h"
#include "itkBinaryImageToShapeLabelMapFilter.h"
#include "itkLabelMapToBinaryImageFilter.h"
#include "itkBinaryImageToStatisticsLabelMapFilter.h"
#include "itkLabelMapToBinaryImageFilter.h"

//距离变换
#include "itkSignedDanielssonDistanceMapImageFilter.h"   //欧氏距离
#include "itkApproximateSignedDistanceMapImageFilter.h"  //近似距离
#include "itkSignedMaurerDistanceMapImageFilter.h"       //莫拉尔距离


#include <iostream>
#include<ctime>
#include <Windows.h>



std::string UtfToGbk(std::string strValue);



const unsigned int ImgDim = 3;				//图像维度
typedef itk::ImageIOBase::IOComponentEnum itkDataType;


typedef itk::Image<float, ImgDim> ImageFloatType;            //数据类型
typedef itk::Image<unsigned short, ImgDim> ImageUshortType;    //16位无符号
typedef itk::Image<unsigned char, ImgDim> ImageUcharType;    //uint8数据类型

typedef ImageFloatType::IndexType IndexFloatType;      //体素坐标
typedef ImageFloatType::SizeType SizeFloatType;        //体素尺寸

//读取raw数据时的IO参数设定
typedef itk::RawImageIO<float, ImgDim> ImageFloatIOType;   
typedef itk::RawImageIO<unsigned short, ImgDim> ImageUshortIOType;
typedef itk::RawImageIO<unsigned char, ImgDim> ImageUcharIOType;


//数据读写类
typedef itk::ImageFileReader<ImageFloatType> ImageRFloatType;
typedef itk::ImageFileReader<ImageUshortType> ImageRUshortType;
typedef itk::ImageFileReader<ImageUcharType> ImageRUcharType;

typedef itk::ImageFileWriter<ImageFloatType> ImageWFloatType;
typedef itk::ImageFileWriter<ImageUshortType> ImageWUshortType;
typedef itk::ImageFileWriter<ImageUcharType> ImageWUcharType;


//形态学部分
typedef itk::BinaryBallStructuringElement< float, ImgDim  > ElementBallFloatType;      //球型区域
typedef itk::BinaryBallStructuringElement< unsigned char, ImgDim  > ElementBallUcharType;      //球型区域
typedef itk::GrayscaleErodeImageFilter <ImageFloatType, ImageFloatType, ElementBallFloatType>GrayErodeType;  //腐蚀
typedef itk::GrayscaleErodeImageFilter <ImageUshortType, ImageUshortType, ElementBallUcharType>GrayErode16Type;  //腐蚀
typedef itk::BlackTopHatImageFilter<ImageFloatType, ImageFloatType, ElementBallFloatType > BlackHatType;     //底帽
typedef itk::BinaryMorphologicalOpeningImageFilter <ImageUcharType, ImageUcharType, ElementBallUcharType> BinaryOpenType;


//分割部分
typedef itk::OtsuThresholdImageFilter <ImageFloatType, ImageUcharType >  OTSUType;        //OTSU
typedef itk::OtsuThresholdImageFilter <ImageUshortType, ImageUcharType >  OTSU16Type;        //OTSU
using ThresholdType = itk::ThresholdImageFilter<ImageFloatType>;     //阈值
using Threshold16Type = itk::ThresholdImageFilter<ImageUshortType>;     //阈值
using BinaryThreshold16Type = itk::BinaryThresholdImageFilter<ImageUshortType, ImageUcharType>;
using BinaryThreshold32Type = itk::BinaryThresholdImageFilter<ImageFloatType, ImageUcharType>;


//图像运算部分
typedef itk::MultiplyImageFilter <ImageFloatType, ImageFloatType >	MultiplyType;
typedef itk::SubtractImageFilter <ImageUshortType, ImageUshortType > Subtract16Type;
typedef itk::AndImageFilter <ImageUcharType, ImageUcharType > And8Type;


//数据类型转换
typedef itk::CastImageFilter< ImageUshortType, ImageFloatType > Ushort2FloatType;
typedef itk::CastImageFilter< ImageFloatType, ImageUshortType > Float2UshortType;

//数据范围转换
typedef itk::RescaleIntensityImageFilter<ImageFloatType, ImageFloatType> RescaleIntensityImageFilterType;
typedef itk::RescaleIntensityImageFilter<ImageUshortType, ImageUshortType> RescaleIntensityImageFilter16Type;

//均值滤波
typedef itk::MeanImageFilter<ImageFloatType, ImageFloatType > MeanFilterType;
typedef itk::MeanImageFilter<ImageUshortType, ImageUshortType > MeanFilter16Type;

//ROI获取
typedef itk::RegionOfInterestImageFilter< ImageFloatType,ImageFloatType > ROIType;
typedef itk::RegionOfInterestImageFilter< ImageUshortType, ImageUshortType > ROI16Type;


//连通域分析
//typedef itk::ShapeLabelObject<unsigned char, ImgDim> LabelObjectType;
typedef itk::ShapeLabelObject<unsigned long int, ImgDim> LabelObjectType;
typedef itk::LabelMap< LabelObjectType> LabelMapType;


typedef itk::BinaryImageToShapeLabelMapFilter< ImageUcharType, LabelMapType >	BinaryI2LType;
typedef itk::LabelMapToBinaryImageFilter<LabelMapType, ImageUcharType> L2BinaryIType;

typedef itk::ConnectedComponentImageFilter<ImageUcharType, ImageUshortType> ConnectedComponentType;
typedef itk::LabelImageToShapeLabelMapFilter<ImageUcharType, LabelMapType> I2LType;

//距离变换
typedef itk::SignedDanielssonDistanceMapImageFilter <ImageUcharType, ImageFloatType> DistanceMapImageType;
typedef itk::SignedMaurerDistanceMapImageFilter <ImageUcharType, ImageFloatType> MaurerDistanceMapImageType;
typedef itk::ApproximateSignedDistanceMapImageFilter <ImageUcharType, ImageFloatType> ApproDistanceMapImageType;



ImageFloatType::Pointer readFloatRawData(std::string dataPath, itkDataType type,
	unsigned int width, unsigned int height, unsigned int deep,
	size_t offset=0);

ImageUshortType::Pointer readUShortRawData(std::string dataPath, itkDataType type,
	unsigned int width, unsigned int height, unsigned int deep,
	size_t offset);

void saveRawData(ImageFloatType::Pointer image, std::string dataPath,
	unsigned int width, unsigned int height, unsigned int deep,
	unsigned int offset = 0); 

void saveRawData16(ImageUshortType::Pointer image, std::string dataPath,
	unsigned int width, unsigned int height, unsigned int deep,
	unsigned int offset = 0);

void saveRawData8(ImageUcharType::Pointer image, std::string dataPath,
	unsigned int width, unsigned int height, unsigned int deep,
	unsigned int offset = 0);

ImageUcharType::Pointer getOtsuMask(ImageFloatType::Pointer image, float& threshold);
ImageUcharType::Pointer getOtsuMask16(ImageUshortType::Pointer image, float& threshold);

ImageFloatType::Pointer getErodeGray(ImageFloatType::Pointer image,int radius);

ImageUshortType::Pointer getErodeGray16(ImageUshortType::Pointer image, int radius);

ImageUcharType::Pointer getOpen8(ImageUcharType::Pointer image, int radius);

ImageFloatType::Pointer getArrayMultiply(ImageFloatType::Pointer image1, ImageFloatType::Pointer image2);

ImageFloatType::Pointer getArrayMultiply(ImageFloatType::Pointer image1, float number2);
ImageUshortType::Pointer getArraySub16(ImageUshortType::Pointer image1, int number2);

ImageUcharType::Pointer getAndImage8(ImageUcharType::Pointer image1, ImageUcharType::Pointer image2);



ImageFloatType::Pointer getMeanFilter(ImageFloatType::Pointer image, int radius);
ImageUshortType::Pointer getMean16Filter(ImageUshortType::Pointer image, int radius);



//数值映射
ImageFloatType::Pointer getRescale(ImageFloatType::Pointer image,float maxValue,float minValue);
ImageUshortType::Pointer getRescale16(ImageUshortType::Pointer image, float maxValue, float minValue);

//阈值分割  ，小于给定值的数置为 changeValue
ImageFloatType::Pointer getThreshold(ImageFloatType::Pointer image,float minValue,float changeValue);
ImageUshortType::Pointer getThreshold16(ImageUshortType::Pointer image,
	float minValue, float changeValue);

//二值化图像距离映射
ImageFloatType::Pointer getDistanceMap(ImageUcharType::Pointer image);


//门限分割
ImageUcharType::Pointer getBinaryMask16(ImageUshortType::Pointer image,
	int lowThres, int highThres, int InsideValue, int OutsideValue);

ImageUcharType::Pointer getBinaryMask32(ImageFloatType::Pointer image,
	int lowThres, int highThres, int InsideValue, int OutsideValue);

ImageFloatType::Pointer getFrangiBlur(ImageFloatType::Pointer image,
	float lambada1, float lambada2, float lambada3);

ImageUshortType::Pointer getFrangiBlur16(ImageUshortType::Pointer image,
	float lambada1, float lambada2, float lambada3);

//多尺度Hessian增强
ImageFloatType::Pointer getHessianEnhence(ImageFloatType::Pointer image,
	double alpha, float beta, float gamma,
	double sigmaMinimum, double sigmaMaximum, unsigned int numberOfSigmaSteps
	);

//ROI获取
ImageFloatType::Pointer getROIdata(ImageFloatType::Pointer image,
	ImageFloatType::IndexType initIndex, ImageFloatType::SizeType roiSize);

ImageUshortType::Pointer getROIdata16(ImageUshortType::Pointer image,
	ImageUshortType::IndexType initIndex, ImageUshortType::SizeType roiSize);

//输出图像尺寸
ImageFloatType::SizeType getSize(ImageFloatType::Pointer image);
ImageUshortType::SizeType getSize(ImageUshortType::Pointer image);
