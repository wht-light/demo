#pragma once

//�ļ���ȡ
#include "itkImage.h"                   //������
#include "itkImageFileReader.h"         
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "itkRawImageIO.h"

#include <itkMultiThreaderBase.h>    //���߳�
#include <itkMultiThreaderBase.h> 


//��̬ѧ����
#include "itkFlatStructuringElement.h"         //�����ض���״��С�Ľṹ����ʮ�֡����Ρ����Ρ������
#include "itkBinaryBallStructuringElement.h"   //��������
#include "itkBinaryCrossStructuringElement.h"  ///������
#include "itkBinaryMorphologicalOpeningImageFilter.h"//������
#include "itkBinaryMorphologicalClosingImageFilter.h"//������
//#include <itkGrayscaleConnectedClosingImageFilter.h>  //�Ҷȱ�����   ����
#include "itkGrayscaleErodeImageFilter.h"//�Ҷȸ�ʴ
#include "itkGrayscaleDilateImageFilter.h"//�Ҷ�����
#include "itkBinaryDilateImageFilter.h"//��ֵ����
#include "itkBinaryErodeImageFilter.h"//��ֵ��ʴ
#include "itkBlackTopHatImageFilter.h"   //��ñ
#include "itkWhiteTopHatImageFilter.h"   //��ñ


#include "itkBinaryThresholdImageFilter.h"  //��ֵ��ͷ�ļ� 
#include "itkOtsuThresholdImageFilter.h"    //Otsu�ָ�ͷ���ļ�
#include "itkThresholdImageFilter.h"        //General Threshold ��Χ�ָ�
#include "itkBinaryThresholdImageFilter.h"  //������ֵ�ָ�


#include "itkMeanImageFilter.h"  //��ֵ�˲�

#include "itkMultiplyImageFilter.h"    //ͼ�����
#include "itkSubtractImageFilter.h"    //��
#include "itkAndImageFilter.h"//��

//hessian�ָ�
#include "itkHessian3DToVesselnessMeasureImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkHessianToObjectnessMeasureImageFilter.h"
#include "itkMultiScaleHessianBasedMeasureImageFilter.h"   //��߶�hessian�˲�

#include "itkCastImageFilter.h"    //��������ת��
#include "itkRescaleIntensityImageFilter.h"   //��ֵӳ��
#include "itkRegionOfInterestImageFilter.h"   //��ȡROI����

//��ͨ�����
#include "itkLabelImageToShapeLabelMapFilter.h"   
#include "itkConnectedComponentImageFilter.h"
#include "itkStatisticsLabelObject.h"
#include "itkBinaryImageToShapeLabelMapFilter.h"
#include "itkLabelMapToBinaryImageFilter.h"
#include "itkBinaryImageToStatisticsLabelMapFilter.h"
#include "itkLabelMapToBinaryImageFilter.h"

//����任
#include "itkSignedDanielssonDistanceMapImageFilter.h"   //ŷ�Ͼ���
#include "itkApproximateSignedDistanceMapImageFilter.h"  //���ƾ���
#include "itkSignedMaurerDistanceMapImageFilter.h"       //Ī��������


#include <iostream>
#include<ctime>
#include <Windows.h>



std::string UtfToGbk(std::string strValue);



const unsigned int ImgDim = 3;				//ͼ��ά��
typedef itk::ImageIOBase::IOComponentEnum itkDataType;


typedef itk::Image<float, ImgDim> ImageFloatType;            //��������
typedef itk::Image<unsigned short, ImgDim> ImageUshortType;    //16λ�޷���
typedef itk::Image<unsigned char, ImgDim> ImageUcharType;    //uint8��������

typedef ImageFloatType::IndexType IndexFloatType;      //��������
typedef ImageFloatType::SizeType SizeFloatType;        //���سߴ�

//��ȡraw����ʱ��IO�����趨
typedef itk::RawImageIO<float, ImgDim> ImageFloatIOType;   
typedef itk::RawImageIO<unsigned short, ImgDim> ImageUshortIOType;
typedef itk::RawImageIO<unsigned char, ImgDim> ImageUcharIOType;


//���ݶ�д��
typedef itk::ImageFileReader<ImageFloatType> ImageRFloatType;
typedef itk::ImageFileReader<ImageUshortType> ImageRUshortType;
typedef itk::ImageFileReader<ImageUcharType> ImageRUcharType;

typedef itk::ImageFileWriter<ImageFloatType> ImageWFloatType;
typedef itk::ImageFileWriter<ImageUshortType> ImageWUshortType;
typedef itk::ImageFileWriter<ImageUcharType> ImageWUcharType;


//��̬ѧ����
typedef itk::BinaryBallStructuringElement< float, ImgDim  > ElementBallFloatType;      //��������
typedef itk::BinaryBallStructuringElement< unsigned char, ImgDim  > ElementBallUcharType;      //��������
typedef itk::GrayscaleErodeImageFilter <ImageFloatType, ImageFloatType, ElementBallFloatType>GrayErodeType;  //��ʴ
typedef itk::GrayscaleErodeImageFilter <ImageUshortType, ImageUshortType, ElementBallUcharType>GrayErode16Type;  //��ʴ
typedef itk::BlackTopHatImageFilter<ImageFloatType, ImageFloatType, ElementBallFloatType > BlackHatType;     //��ñ
typedef itk::BinaryMorphologicalOpeningImageFilter <ImageUcharType, ImageUcharType, ElementBallUcharType> BinaryOpenType;


//�ָ��
typedef itk::OtsuThresholdImageFilter <ImageFloatType, ImageUcharType >  OTSUType;        //OTSU
typedef itk::OtsuThresholdImageFilter <ImageUshortType, ImageUcharType >  OTSU16Type;        //OTSU
using ThresholdType = itk::ThresholdImageFilter<ImageFloatType>;     //��ֵ
using Threshold16Type = itk::ThresholdImageFilter<ImageUshortType>;     //��ֵ
using BinaryThreshold16Type = itk::BinaryThresholdImageFilter<ImageUshortType, ImageUcharType>;
using BinaryThreshold32Type = itk::BinaryThresholdImageFilter<ImageFloatType, ImageUcharType>;


//ͼ�����㲿��
typedef itk::MultiplyImageFilter <ImageFloatType, ImageFloatType >	MultiplyType;
typedef itk::SubtractImageFilter <ImageUshortType, ImageUshortType > Subtract16Type;
typedef itk::AndImageFilter <ImageUcharType, ImageUcharType > And8Type;


//��������ת��
typedef itk::CastImageFilter< ImageUshortType, ImageFloatType > Ushort2FloatType;
typedef itk::CastImageFilter< ImageFloatType, ImageUshortType > Float2UshortType;

//���ݷ�Χת��
typedef itk::RescaleIntensityImageFilter<ImageFloatType, ImageFloatType> RescaleIntensityImageFilterType;
typedef itk::RescaleIntensityImageFilter<ImageUshortType, ImageUshortType> RescaleIntensityImageFilter16Type;

//��ֵ�˲�
typedef itk::MeanImageFilter<ImageFloatType, ImageFloatType > MeanFilterType;
typedef itk::MeanImageFilter<ImageUshortType, ImageUshortType > MeanFilter16Type;

//ROI��ȡ
typedef itk::RegionOfInterestImageFilter< ImageFloatType,ImageFloatType > ROIType;
typedef itk::RegionOfInterestImageFilter< ImageUshortType, ImageUshortType > ROI16Type;


//��ͨ�����
//typedef itk::ShapeLabelObject<unsigned char, ImgDim> LabelObjectType;
typedef itk::ShapeLabelObject<unsigned long int, ImgDim> LabelObjectType;
typedef itk::LabelMap< LabelObjectType> LabelMapType;


typedef itk::BinaryImageToShapeLabelMapFilter< ImageUcharType, LabelMapType >	BinaryI2LType;
typedef itk::LabelMapToBinaryImageFilter<LabelMapType, ImageUcharType> L2BinaryIType;

typedef itk::ConnectedComponentImageFilter<ImageUcharType, ImageUshortType> ConnectedComponentType;
typedef itk::LabelImageToShapeLabelMapFilter<ImageUcharType, LabelMapType> I2LType;

//����任
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



//��ֵӳ��
ImageFloatType::Pointer getRescale(ImageFloatType::Pointer image,float maxValue,float minValue);
ImageUshortType::Pointer getRescale16(ImageUshortType::Pointer image, float maxValue, float minValue);

//��ֵ�ָ�  ��С�ڸ���ֵ������Ϊ changeValue
ImageFloatType::Pointer getThreshold(ImageFloatType::Pointer image,float minValue,float changeValue);
ImageUshortType::Pointer getThreshold16(ImageUshortType::Pointer image,
	float minValue, float changeValue);

//��ֵ��ͼ�����ӳ��
ImageFloatType::Pointer getDistanceMap(ImageUcharType::Pointer image);


//���޷ָ�
ImageUcharType::Pointer getBinaryMask16(ImageUshortType::Pointer image,
	int lowThres, int highThres, int InsideValue, int OutsideValue);

ImageUcharType::Pointer getBinaryMask32(ImageFloatType::Pointer image,
	int lowThres, int highThres, int InsideValue, int OutsideValue);

ImageFloatType::Pointer getFrangiBlur(ImageFloatType::Pointer image,
	float lambada1, float lambada2, float lambada3);

ImageUshortType::Pointer getFrangiBlur16(ImageUshortType::Pointer image,
	float lambada1, float lambada2, float lambada3);

//��߶�Hessian��ǿ
ImageFloatType::Pointer getHessianEnhence(ImageFloatType::Pointer image,
	double alpha, float beta, float gamma,
	double sigmaMinimum, double sigmaMaximum, unsigned int numberOfSigmaSteps
	);

//ROI��ȡ
ImageFloatType::Pointer getROIdata(ImageFloatType::Pointer image,
	ImageFloatType::IndexType initIndex, ImageFloatType::SizeType roiSize);

ImageUshortType::Pointer getROIdata16(ImageUshortType::Pointer image,
	ImageUshortType::IndexType initIndex, ImageUshortType::SizeType roiSize);

//���ͼ��ߴ�
ImageFloatType::SizeType getSize(ImageFloatType::Pointer image);
ImageUshortType::SizeType getSize(ImageUshortType::Pointer image);
