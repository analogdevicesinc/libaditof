/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef FRAME_IMPL
#define FRAME_IMPL

#include <aditof/frame_definitions.h>
#include <aditof/status_definitions.h>

#include <map>
#include <memory>
#include <stdint.h>
#include <vector>

class FrameImpl {
  public:
    FrameImpl();
    //~FrameImpl();
    FrameImpl(const FrameImpl &op);
    FrameImpl &operator=(const FrameImpl &op);
    ~FrameImpl();

  public: // from TofFrame
    aditof::Status setDetails(const aditof::FrameDetails &details,
                              const uint8_t &m_bitsInConf,
                              const uint8_t &m_bitsInAB);
    aditof::Status getDetails(aditof::FrameDetails &details) const;
    aditof::Status getDataDetails(const std::string &dataType,
                                  aditof::FrameDataDetails &details) const;
    aditof::Status getData(const std::string &dataType, uint16_t **dataPtr);
    aditof::Status getMetadataStruct(aditof::Metadata &metadata) const;
    bool haveDataType(const std::string &dataType);

  private:
    void allocFrameData(const aditof::FrameDetails &details);

  private:
    struct ImplData;
    aditof::FrameDetails m_details;
    std::unique_ptr<ImplData> m_implData;
    aditof::FrameDataDetails
    getFrameDetailByName(const aditof::FrameDetails &details,
                         const std::string name);
};

#endif // FRAME_IMPL
