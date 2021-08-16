/*-
 * Copyright (c) 2020 Nathan Lay (enslay@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Common.h"
#include "bsdgetopt.h"

// ITK stuff
#include "itkImageIOBase.h"
#include "itkImageIOFactory.h"
#include "itkGDCMImageIO.h"
#include "itkResampleImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkNearestNeighborInterpolateImageFunction.h"

#if ITK_VERSION_MAJOR > 4
using IOFileModeEnum = itk::IOFileModeEnum;
using IOComponentEnum = itk::IOComponentEnum;
using IOPixelEnum = itk::IOPixelEnum;
#else // ITK_VERSION_MAJOR <= 4
using IOFileModeEnum = itk::ImageIOFactory;
using IOComponentEnum = itk::ImageIOBase;
using IOPixelEnum = itk::ImageIOBase;
#endif // ITK_VERSION_MAJOR > 4

void Usage(const char *p_cArg0) {
  std::cerr << "Usage: " << p_cArg0 << " [-ch] [-r resolution] [-s size] [-o outputFolder] [-i nearest|linear] targetVolume sourceVolume [sourceVolume2 ...]" << std::endl;
  std::cerr << "\nOptions:" << std::endl;
  std::cerr << "-c -- Compress output images." << std::endl;
  std::cerr << "-h -- This help message." << std::endl;
  std::cerr << "-i -- Interpolator type (default: nearest). May be one of 'nearest' or 'linear'." << std::endl;
  std::cerr << "-o -- Output folder where aligned volumes are saved (default: output)." << std::endl;
  std::cerr << "-r -- Set voxel spacing to be XxYxZ where X, Y and Z specify the new X, Y and Z axis spacings (format 'XxYxZ'). These values may be 0 or negative to keep the corresponding input target volume X, Y or Z spacings." << std::endl;
  std::cerr << "-s -- Set image dimensions to be XxYxZ where X, Y and Z specify the new X, Y and Z dimensions (format 'XxYxZ'). These values may be 0 to keep the corresponding input target volume X, Y or Z dimensions." << std::endl;
  std::cerr << "\nNOTE: Only one of -r or -s may be specified." << std::endl;
  exit(1);
}

template<typename PixelType>
struct ZeroPixel {
  static PixelType Value() { return PixelType(); }
};

template<typename ComponentType>
struct ZeroPixel<itk::RGBPixel<ComponentType>> {
  typedef itk::RGBPixel<ComponentType> PixelType;
  static PixelType Value() { 
    PixelType clPixel;
    clPixel.Fill(ComponentType());
    return clPixel;
  }
};

template<typename ComponentType>
struct ZeroPixel<itk::RGBAPixel<ComponentType>> {
  typedef itk::RGBAPixel<ComponentType> PixelType;
  static PixelType Value() { 
    PixelType clPixel;
    clPixel.Fill(ComponentType());
    return clPixel;
  }
};

template<typename ComponentType, unsigned int Dimension>
struct ZeroPixel<itk::Vector<ComponentType, Dimension>> {
  typedef itk::Vector<ComponentType, Dimension> PixelType;
  static PixelType Value() { 
    PixelType clPixel;
    clPixel.Fill(ComponentType());
    return clPixel;
  }
};

// Crude image resampler
class ImageResamplerBase {
public:
  typedef itk::ImageBase<3> ImageType;
  typedef ImageType::DirectionType DirectionType;
  typedef ImageType::SpacingType SpacingType;
  typedef ImageType::PointType PointType;
  typedef ImageType::SizeType SizeType;
  typedef itk::Index<3> IndexType;
  enum InterpolationType { NEAREST, LINEAR };

  virtual ~ImageResamplerBase() = default;

  virtual bool Good() const = 0;
  virtual bool LoadImg(const std::string &strImagePath) = 0;
  virtual bool SaveImg(const std::string &strImagePath, bool bCompress) = 0;

  virtual bool Resample() = 0;

  virtual const DirectionType & GetInputDirection() const = 0;
  virtual const SpacingType & GetInputSpacing() const = 0;
  virtual const PointType & GetInputOrigin() const = 0;
  virtual const SizeType & GetInputSize() const = 0;

  virtual const DirectionType & GetOutputDirection() const { return m_clOutputDirection; }
  virtual const SpacingType & GetOutputSpacing() const { return m_clOutputSpacing; }
  virtual const PointType & GetOutputOrigin() const { return m_clOutputOrigin; }
  virtual const SizeType & GetOutputSize() const { return m_clOutputSize; }

  virtual void SetOutputDirection(const DirectionType &clOutputDirection) { m_clOutputDirection = clOutputDirection; }
  virtual void SetOutputSpacing(const SpacingType &clOutputSpacing) { m_clOutputSpacing = clOutputSpacing; }
  virtual void SetOutputOrigin(const PointType &clOutputOrigin) { m_clOutputOrigin = clOutputOrigin; }
  virtual void SetOutputSize(const SizeType &clOutputSize) { m_clOutputSize = clOutputSize; }

  virtual void SetInterpolationType(InterpolationType eInterpolationType) { m_eInterpolationType = eInterpolationType; }
  virtual InterpolationType GetInterpolationType() const { return m_eInterpolationType; }

  virtual bool SetInterpolationType(const std::string &strInterpolationType) {
    if (strInterpolationType == "nearest")
      SetInterpolationType(NEAREST);
    else if (strInterpolationType == "linear")
      SetInterpolationType(LINEAR);
    else
      return false;

    return true;
  }

private:
  DirectionType m_clOutputDirection;
  SpacingType m_clOutputSpacing;
  PointType m_clOutputOrigin;
  SizeType m_clOutputSize;
  InterpolationType m_eInterpolationType = NEAREST;
};

template<typename PixelType>
class ImageResampler : public ImageResamplerBase {
public:
  typedef itk::Image<PixelType, 3> ImageType;
  typedef itk::ResampleImageFilter<ImageType, ImageType> ResamplerType;
  typedef itk::InterpolateImageFunction<ImageType> InterpolatorType;
  typedef itk::NearestNeighborInterpolateImageFunction<ImageType> NearestNeigborInterpolationType;
  typedef itk::LinearInterpolateImageFunction<ImageType> LinearInterpolationType;

  virtual ~ImageResampler() = default;

  virtual bool Good() const override { return m_p_clInputImage.IsNotNull(); }

  virtual bool LoadImg(const std::string &strImagePath) override {
    if (IsFolder(strImagePath) || GetExtension(strImagePath) == ".dcm")
      m_p_clInputImage = ::LoadDicomImage<PixelType, 3>(strImagePath);
    else
      m_p_clInputImage = ::LoadImg<PixelType, 3>(strImagePath);

    if (!m_p_clInputImage)
      return false;

    // Setup default resampling parameters
    SetOutputDirection(GetInputDirection());
    SetOutputSpacing(GetInputSpacing());
    SetOutputOrigin(GetInputOrigin());
    SetOutputSize(GetInputSize());

    return true;
  }

  virtual bool SaveImg(const std::string &strImagePath, bool bCompress) override {
    if (!m_p_clOutputImage || !m_p_clInputImage)
      return false;

    if (GetExtension(strImagePath).empty()) {
      m_p_clOutputImage->SetMetaDataDictionary(m_p_clInputImage->GetMetaDataDictionary());
      return ::SaveDicomImage(m_p_clOutputImage.GetPointer(), strImagePath, bCompress);
    }

    return ::SaveImg(m_p_clOutputImage.GetPointer(), strImagePath, bCompress);
  }

  virtual bool Resample() override {
    m_p_clOutputImage = nullptr;

    if (!Good())
      return false;

    typename ResamplerType::Pointer p_clResampler = ResamplerType::New();

    p_clResampler->SetDefaultPixelValue(ZeroPixel<PixelType>::Value());
    p_clResampler->SetSize(GetOutputSize());
    p_clResampler->SetOutputSpacing(GetOutputSpacing());
    p_clResampler->SetOutputOrigin(GetOutputOrigin());
    p_clResampler->SetOutputDirection(GetOutputDirection());

    typename InterpolatorType::Pointer p_clInterpolator;

    switch (GetInterpolationType()) {
    case NEAREST:
      p_clInterpolator = NearestNeigborInterpolationType::New();
      break;
    case LINEAR:
      p_clInterpolator = LinearInterpolationType::New();
      break;
    }

    if (!p_clInterpolator)
      return false;

    p_clResampler->SetInterpolator(p_clInterpolator);
    p_clResampler->SetInput(m_p_clInputImage);

    try {
      p_clResampler->Update();
      m_p_clOutputImage = p_clResampler->GetOutput();
    }
    catch (itk::ExceptionObject &e) {
      std::cerr << "Error: Failed to resample image: " << e.what() << std::endl;
      return false;
    }

    return true;
  }

  virtual const DirectionType & GetInputDirection() const override { return m_p_clInputImage->GetDirection(); }
  virtual const SpacingType & GetInputSpacing() const override { return m_p_clInputImage->GetSpacing(); }
  virtual const PointType & GetInputOrigin() const override { return m_p_clInputImage->GetOrigin(); }
  virtual const SizeType & GetInputSize() const override { return m_p_clInputImage->GetBufferedRegion().GetSize(); }

private:
  typename ImageType::Pointer m_p_clInputImage;
  typename ImageType::Pointer m_p_clOutputImage;
};

std::unique_ptr<ImageResamplerBase> MakeResampler(const std::string &strImagePath);

std::string MakeOutputPath(const std::string &strInputImagePath, const std::string &strOutputImageFolder) {
  if (IsFolder(strInputImagePath)) {
    return strOutputImageFolder + '/' + BaseName(strInputImagePath);
  }
  else if (GetExtension(strInputImagePath) == ".dcm")
    return strOutputImageFolder + '/' + DirName(strInputImagePath);

  return strOutputImageFolder + '/' + BaseName(strInputImagePath);
}

int main(int argc, char **argv) {
  const char * const p_cArg0 = argv[0];

  std::string strOutputImageFolder = "output";
  std::string strInterpolatorType = "nearest";

  ImageResamplerBase::SpacingType clNewSpacing;
  clNewSpacing.Fill(0.0);

  ImageResamplerBase::SizeType clNewSize;
  clNewSize.Fill(0);

  bool bCompress = false;

  int c = 0;
  while ((c = getopt(argc, argv, "chi:o:r:s:")) != -1) {
    switch (c) {
    case 'c':
      bCompress = true;
      break;
    case 'h':
      Usage(p_cArg0);
      break;
    case 'i':
      strInterpolatorType = optarg;
      break;
    case 'o':
      strOutputImageFolder = optarg;
      break;
    case 'r':
      {
        char *p = nullptr;

        clNewSpacing[0] = strtod(optarg, &p);

        if (*p != 'x' || *(p+1) == 'x' || *(p+1) == '\0')
          Usage(p_cArg0);

        clNewSpacing[1] = strtod(p+1, &p);

        if (*p != 'x' || *(p+1) == 'x' || *(p+1) == '\0')
          Usage(p_cArg0);

        clNewSpacing[2] = strtod(p+1, &p);

        if (*p != '\0')
          Usage(p_cArg0);
      }
      break;
    case 's':
      {
        char *p = nullptr;

        clNewSize[0] = strtoul(optarg, &p, 10);

        if (*p != 'x' || *(p+1) == 'x' || *(p+1) == '\0')
          Usage(p_cArg0);

        clNewSize[1] = strtoul(p+1, &p, 10);

        if (*p != 'x' || *(p+1) == 'x' || *(p+1) == '\0')
          Usage(p_cArg0);

        clNewSize[2] = strtoul(p+1, &p, 10);

        if (*p != '\0')
          Usage(p_cArg0);
      }
      break;
    case '?':
    default:
      Usage(p_cArg0);
      break;
    }
  }

  argc -= optind;
  argv += optind;

  if (argc < 2) {
    std::cerr << "Error: Need at least two volumes to align." << std::endl;
    Usage(p_cArg0);
  }

  const bool bNewResolution = (clNewSpacing[0] > 0.0 || clNewSpacing[1] > 0.0 || clNewSpacing[2] > 0.0);
  const bool bNewSize = (clNewSize[0] > 0 || clNewSize[1] > 0 || clNewSize[2] > 0);

  if (bNewResolution && bNewSize)
    Usage(p_cArg0);

  std::cout << "Info: Loading target '" << argv[0] << "' ..." << std::endl;
  std::unique_ptr<ImageResamplerBase> p_clTarget = MakeResampler(argv[0]);

  if (!p_clTarget) {
    std::cerr << "Error: Failed to make target resampler for image '" << argv[0] << "'." << std::endl;
    return -1;
  }

  if (MkDir(strOutputImageFolder))
    std::cout << "Info: Made folder '" << strOutputImageFolder << "'." << std::endl;

  if (bNewResolution) {
    ImageResamplerBase::SpacingType clOutputSpacing = p_clTarget->GetInputSpacing();
    ImageResamplerBase::SizeType clOutputSize = p_clTarget->GetInputSize();

    if (clNewSpacing[0] > 0.0) {
      clOutputSize[0] = itk::SizeValueType(clOutputSize[0]*clOutputSpacing[0]/clNewSpacing[0] + 0.5);
      clOutputSpacing[0] = clNewSpacing[0];
    }

    if (clNewSpacing[1] > 0.0) {
      clOutputSize[1] = itk::SizeValueType(clOutputSize[1]*clOutputSpacing[1]/clNewSpacing[1] + 0.5);
      clOutputSpacing[1] = clNewSpacing[1];
    }

    if (clNewSpacing[2] > 0.0) {
      clOutputSize[2] = itk::SizeValueType(clOutputSize[2]*clOutputSpacing[2]/clNewSpacing[2] + 0.5);
      clOutputSpacing[2] = clNewSpacing[2];
    }

    std::cout << "Info: Resampling all volumes to have spacing " << clOutputSpacing << " and size " << clOutputSize << '.' << std::endl;

    p_clTarget->SetOutputSpacing(clOutputSpacing);
    p_clTarget->SetOutputSize(clOutputSize);
  }

  if (bNewSize) {
    ImageResamplerBase::SpacingType clOutputSpacing = p_clTarget->GetInputSpacing();
    ImageResamplerBase::SizeType clOutputSize = p_clTarget->GetInputSize();

    if (clNewSize[0] > 0) {
      clOutputSpacing[0] = itk::SpacePrecisionType(clOutputSize[0]*clOutputSpacing[0]/clNewSize[0]);
      clOutputSize[0] = clNewSize[0];
    }

    if (clNewSize[1] > 0) {
      clOutputSpacing[1] = itk::SpacePrecisionType(clOutputSize[1]*clOutputSpacing[1]/clNewSize[1]);
      clOutputSize[1] = clNewSize[1];
    }

    if (clNewSize[2] > 0) {
      clOutputSpacing[2] = itk::SpacePrecisionType(clOutputSize[2]*clOutputSpacing[2]/clNewSize[2]);
      clOutputSize[2] = clNewSize[2];
    }

    std::cout << "Info: Resampling all volumes to have spacing " << clOutputSpacing << " and size " << clOutputSize << '.' << std::endl;

    p_clTarget->SetOutputSpacing(clOutputSpacing);
    p_clTarget->SetOutputSize(clOutputSize);
  }

  if (!p_clTarget->SetInterpolationType(strInterpolatorType)) {
    std::cerr << "Error: Unrecognized interpolation type '" << strInterpolatorType << "'." << std::endl;
    return -1;
  }

  if (bNewResolution || bNewSize) {
    std::cout << "Info: Resampling target volume ..." << std::endl;

    if (!p_clTarget->Resample()) {
      std::cerr<< "Error: Failed to resample target volume." << std::endl;
      return -1;
    }

    const std::string strOutputImagePath = MakeOutputPath(argv[0], strOutputImageFolder);

    std::cout << "Info: Saving '" << strOutputImagePath << "' ..." << std::endl;

    if (!p_clTarget->SaveImg(strOutputImagePath, bCompress)) {
      std::cerr << "Error: Failed to save image '" << strOutputImagePath << "'." << std::endl;
      return -1;
    }
  }

  for (int i = 1; i < argc; ++i) {
    const std::string strInputImagePath = argv[i];

    std::cout << "Info: Loading source '" << strInputImagePath << "' ..." << std::endl;

    std::unique_ptr<ImageResamplerBase> p_clSource = MakeResampler(strInputImagePath);

    if (!p_clSource) {
      std::cerr << "Error: Failed to make source resampler for image '" << strInputImagePath << "'." << std::endl;
      return -1;
    }

    p_clSource->SetOutputDirection(p_clTarget->GetOutputDirection());
    p_clSource->SetOutputSpacing(p_clTarget->GetOutputSpacing());
    p_clSource->SetOutputOrigin(p_clTarget->GetOutputOrigin());
    p_clSource->SetOutputSize(p_clTarget->GetOutputSize());
    p_clSource->SetInterpolationType(p_clTarget->GetInterpolationType());

    const std::string strOutputImagePath = MakeOutputPath(strInputImagePath, strOutputImageFolder);

    std::cout << "Info: Resampling source volume ..." << std::endl;

    if (!p_clSource->Resample()) {
      std::cerr<< "Error: Failed to resample source volume." << std::endl;
      return -1;
    }

    std::cout << "Info: Saving '" << strOutputImagePath << "' ..." << std::endl;

    if (!p_clSource->SaveImg(strOutputImagePath, bCompress)) {
      std::cerr << "Error: Failed to save image '" << strOutputImagePath << "'." << std::endl;
      return -1;
    }
  }

  return 0;
}

std::unique_ptr<ImageResamplerBase> MakeResampler(const std::string &strImagePath) {
  auto eComponentType = IOComponentEnum::UNKNOWNCOMPONENTTYPE;
  auto ePixelType = IOPixelEnum::UNKNOWNPIXELTYPE;
  unsigned int uiNumberOfComponents = 0;

  if (IsFolder(strImagePath)) {
    std::vector<std::string> vFiles;
    FindDicomFiles(strImagePath.c_str(), "*", vFiles, false);

    if (vFiles.empty())
      return std::unique_ptr<ImageResamplerBase>();

    itk::GDCMImageIO::Pointer p_clImageIO = itk::GDCMImageIO::New();
    p_clImageIO->SetFileName(vFiles[0]);

    try {
      p_clImageIO->ReadImageInformation();

      eComponentType = p_clImageIO->GetInternalComponentType();
      ePixelType = p_clImageIO->GetPixelType();
      uiNumberOfComponents = p_clImageIO->GetNumberOfComponents();
    }
    catch (itk::ExceptionObject &) {
      return std::unique_ptr<ImageResamplerBase>();
    }
  }
  else if (GetExtension(strImagePath) == ".dcm") {
    itk::GDCMImageIO::Pointer p_clImageIO = itk::GDCMImageIO::New();
    p_clImageIO->SetFileName(strImagePath);

    try {
      p_clImageIO->ReadImageInformation();

      eComponentType = p_clImageIO->GetInternalComponentType();
      ePixelType = p_clImageIO->GetPixelType();
      uiNumberOfComponents = p_clImageIO->GetNumberOfComponents();
    }
    catch (itk::ExceptionObject &) {
      return std::unique_ptr<ImageResamplerBase>();
    }
  }
  else {
    itk::ImageIOBase::Pointer p_clImageIO = itk::ImageIOFactory::CreateImageIO(strImagePath.c_str(), IOFileModeEnum::ReadMode);

    if (!p_clImageIO)
      return std::unique_ptr<ImageResamplerBase>();

    p_clImageIO->SetFileName(strImagePath);

    try {
      p_clImageIO->ReadImageInformation();

      eComponentType = p_clImageIO->GetComponentType();
      ePixelType = p_clImageIO->GetPixelType();
      uiNumberOfComponents = p_clImageIO->GetNumberOfComponents();
    }
    catch (itk::ExceptionObject &) {
      return std::unique_ptr<ImageResamplerBase>();
    }
  }

  std::unique_ptr<ImageResamplerBase> p_clResampler;

  switch (ePixelType) {
  case IOPixelEnum::SCALAR:
    switch (eComponentType) {
    case IOComponentEnum::CHAR:
      p_clResampler = std::make_unique<ImageResampler<char>>();
      break;
    case IOComponentEnum::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<unsigned char>>();
      break;
    case IOComponentEnum::SHORT:
      p_clResampler = std::make_unique<ImageResampler<short>>();
      break;
    case IOComponentEnum::USHORT:
      p_clResampler = std::make_unique<ImageResampler<unsigned short>>();
      break;
    case IOComponentEnum::INT:
      p_clResampler = std::make_unique<ImageResampler<int>>();
      break;
    case IOComponentEnum::UINT:
      p_clResampler = std::make_unique<ImageResampler<unsigned int>>();
      break;
    case IOComponentEnum::FLOAT:
      p_clResampler = std::make_unique<ImageResampler<float>>();
      break;
    default:
      break;
    }
    break;
  case IOPixelEnum::RGB:
    switch (eComponentType) {
    case IOComponentEnum::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<itk::RGBPixel<unsigned char>>>();
      break;
    default:
      break;
    }
    break;
  case IOPixelEnum::RGBA:
    switch (eComponentType) {
    case IOComponentEnum::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<itk::RGBAPixel<unsigned char>>>();
      break;
    default:
      break;
    }
    break;
  case IOPixelEnum::VECTOR:
    // TODO: Finish this?
    break;
  default:
    break;
  }

  if (!p_clResampler || !p_clResampler->LoadImg(strImagePath))
    return std::unique_ptr<ImageResamplerBase>();

  return std::move(p_clResampler);
}
