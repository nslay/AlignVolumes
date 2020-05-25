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

void Usage(const char *p_cArg0) {
  std::cerr << "Usage: " << p_cArg0 << " [-h] [-r resolution] -o outputFolder targetVolume sourceVolume [sourceVolume2 ...]" << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "-h -- This help message." << std::endl;
  std::cerr << "-o -- Output folder where aligned volumes are saved." << std::endl;
  std::cerr << "-r -- Set voxel spacing to be XxYxZ where X, Y and Z specify new X, Y and Z axis spacing. These values may be 0 or negative to keep the corresponding input X, Y or Z spacing." << std::endl;
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

  virtual ~ImageResamplerBase() = default;

  virtual bool Good() const = 0;
  virtual bool LoadImg(const std::string &strImagePath) = 0;
  virtual bool SaveImg(const std::string &strImagePath) = 0;
  virtual bool Initialize() = 0;
  virtual void SetPixel(const PointType &clPatientPoint) = 0;
  virtual void SetPixel(const IndexType &clIndex) = 0;

  virtual void Resample() {
    if (!Good())
      return;

    const SizeType clSize = GetOutputSize();

    for (itk::IndexValueType z = 0; itk::SizeValueType(z) < clSize[2]; ++z) {
      for (itk::IndexValueType y = 0; itk::SizeValueType(y) < clSize[1]; ++y) {
        for (itk::IndexValueType x = 0; itk::SizeValueType(x) < clSize[0]; ++x) {
          const IndexType clIndex = {{ x, y, z }};
          SetPixel(clIndex);
        }
      }
    }
  }

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

private:
  DirectionType m_clOutputDirection;
  SpacingType m_clOutputSpacing;
  PointType m_clOutputOrigin;
  SizeType m_clOutputSize;
};

template<typename PixelType>
class ImageResampler : public ImageResamplerBase {
public:
  typedef itk::Image<PixelType, 3> ImageType;

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

  virtual bool SaveImg(const std::string &strImagePath) override {
    if (!m_p_clOutputImage || !m_p_clInputImage)
      return false;

    if (GetExtension(strImagePath).empty()) {
      m_p_clOutputImage->SetMetaDataDictionary(m_p_clInputImage->GetMetaDataDictionary());
      return ::SaveDicomImage(m_p_clOutputImage.GetPointer(), strImagePath);
    }

    return ::SaveImg(m_p_clOutputImage.GetPointer(), strImagePath);
  }

  virtual bool Initialize() override {
    if (!Good())
      return false;

    m_p_clOutputImage = ImageType::New();
    m_p_clOutputImage->SetRegions(GetOutputSize());

    try {
      m_p_clOutputImage->Allocate();
    }
    catch (itk::ExceptionObject &e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return false;
    }

    m_p_clOutputImage->SetDirection(GetOutputDirection());
    m_p_clOutputImage->SetSpacing(GetOutputSpacing());
    m_p_clOutputImage->SetOrigin(GetOutputOrigin());

    m_p_clOutputImage->FillBuffer(ZeroPixel<PixelType>::Value());

    return true;
  }

  virtual void SetPixel(const PointType &clPatientPoint) override {
    if (!m_p_clInputImage || !m_p_clOutputImage)
      return;

    // TODO: Interpolation in the future
    IndexType clInputIndex, clOutputIndex;
    if (!m_p_clInputImage->TransformPhysicalPointToIndex(clPatientPoint, clInputIndex) || !m_p_clOutputImage->TransformPhysicalPointToIndex(clPatientPoint, clOutputIndex))
      return;

    m_p_clOutputImage->SetPixel(clOutputIndex, m_p_clInputImage->GetPixel(clInputIndex));
  }

  virtual void SetPixel(const IndexType &clOutputIndex) override {
    if (!m_p_clInputImage || !m_p_clOutputImage)
      return;

    // TODO: Interpolation in the future    
    if (!m_p_clOutputImage->GetBufferedRegion().IsInside(clOutputIndex))
      return;

    PointType clPatientPoint;
    m_p_clOutputImage->TransformIndexToPhysicalPoint(clOutputIndex, clPatientPoint);

    IndexType clInputIndex;
    if (!m_p_clInputImage->TransformPhysicalPointToIndex(clPatientPoint, clInputIndex))
      return;

    m_p_clOutputImage->SetPixel(clOutputIndex, m_p_clInputImage->GetPixel(clInputIndex));
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

  ImageResamplerBase::SpacingType clNewSpacing;
  clNewSpacing.Fill(0.0);

  int c = 0;
  while ((c = getopt(argc, argv, "ho:r:")) != -1) {
    switch (c) {
    case 'h':
      Usage(p_cArg0);
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

  if (!p_clTarget->Initialize()) {
    std::cerr<< "Error: Failed to initialize target resampler." << std::endl;
    return -1;
  }

  if (bNewResolution) {
    std::cout << "Info: Resampling target volume ..." << std::endl;

    p_clTarget->Resample();

    const std::string strOutputImagePath = MakeOutputPath(argv[0], strOutputImageFolder);

    std::cout << "Info: Saving '" << strOutputImagePath << "' ..." << std::endl;

    if (!p_clTarget->SaveImg(strOutputImagePath)) {
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

    if (!p_clSource->Initialize()) {
      std::cerr << "Error: Failed to initialize source resampler." << std::endl;
      return -1;
    }

    const std::string strOutputImagePath = MakeOutputPath(strInputImagePath, strOutputImageFolder);

    std::cout << "Info: Resampling source volume ..." << std::endl;

    p_clSource->Resample();

    std::cout << "Info: Saving '" << strOutputImagePath << "' ..." << std::endl;

    if (!p_clSource->SaveImg(strOutputImagePath)) {
      std::cerr << "Error: Failed to save image '" << strOutputImagePath << "'." << std::endl;
      return -1;
    }
  }

  return 0;
}

std::unique_ptr<ImageResamplerBase> MakeResampler(const std::string &strImagePath) {
  itk::ImageIOBase::IOComponentType eComponentType = itk::ImageIOBase::UNKNOWNCOMPONENTTYPE;
  itk::ImageIOBase::IOPixelType ePixelType = itk::ImageIOBase::UNKNOWNPIXELTYPE;
  unsigned int uiNumberOfComponents = 0;

  if (IsFolder(strImagePath)) {
    std::vector<std::string> vFiles;
    FindFiles(strImagePath.c_str(), "*", vFiles, false);

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
    itk::ImageIOBase::Pointer p_clImageIO = itk::ImageIOFactory::CreateImageIO(strImagePath.c_str(), itk::ImageIOFactory::ReadMode);

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
  case itk::ImageIOBase::SCALAR:
    switch (eComponentType) {
    case itk::ImageIOBase::CHAR:
      p_clResampler = std::make_unique<ImageResampler<char>>();
      break;
    case itk::ImageIOBase::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<unsigned char>>();
      break;
    case itk::ImageIOBase::SHORT:
      p_clResampler = std::make_unique<ImageResampler<short>>();
      break;
    case itk::ImageIOBase::USHORT:
      p_clResampler = std::make_unique<ImageResampler<unsigned short>>();
      break;
    case itk::ImageIOBase::INT:
      p_clResampler = std::make_unique<ImageResampler<int>>();
      break;
    case itk::ImageIOBase::UINT:
      p_clResampler = std::make_unique<ImageResampler<unsigned int>>();
      break;
    case itk::ImageIOBase::FLOAT:
      p_clResampler = std::make_unique<ImageResampler<float>>();
      break;
    default:
      break;
    }
    break;
  case itk::ImageIOBase::RGB:
    switch (eComponentType) {
    case itk::ImageIOBase::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<itk::RGBPixel<unsigned char>>>();
      break;
    default:
      break;
    }
    break;
  case itk::ImageIOBase::RGBA:
    switch (eComponentType) {
    case itk::ImageIOBase::UCHAR:
      p_clResampler = std::make_unique<ImageResampler<itk::RGBAPixel<unsigned char>>>();
      break;
    default:
      break;
    }
    break;
  case itk::ImageIOBase::VECTOR:
    // TODO: Finish this?
    break;
   default:
    break;
  }

  if (!p_clResampler || !p_clResampler->LoadImg(strImagePath))
    return std::unique_ptr<ImageResamplerBase>();

  return std::move(p_clResampler);
}
