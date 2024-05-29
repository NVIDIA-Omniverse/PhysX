// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef BLASTBASEPERFTEST_H
#define BLASTBASEPERFTEST_H


#include "BlastBaseTest.h"
#include <fstream>

#include <algorithm>
#include <map>


template<typename T>
class DataCollection
{
public:
    struct Stats
    {
        double  m_mean;
        double  m_sdev;
        double  m_min;
        double  m_max;

        Stats()
        {
            reset();
        }

        void reset()
        {
            m_mean = 0.0;
            m_sdev = 0.0;
            m_min = std::numeric_limits<double>().max();
            m_max = -std::numeric_limits<double>().max();
        }
    };

    struct DataSet
    {
        std::vector<T>  m_data;
        Stats           m_stats;

        void    calculateStats()
        {
            m_stats.reset();
            if (m_data.size() > 0)
            {
                if (m_data.size() > 1)  // Remove top half of values to eliminate outliers
                {
                    std::sort(m_data.begin(), m_data.end());
                    m_data.resize(m_data.size() / 2);
                }
                for (size_t i = 0; i < m_data.size(); ++i)
                {
                    m_stats.m_mean += m_data[i];
                    m_stats.m_min = std::min(m_stats.m_min, (double)m_data[i]);
                    m_stats.m_max = std::max(m_stats.m_max, (double)m_data[i]);
                }
                m_stats.m_mean /= m_data.size();
                if (m_data.size() > 1)
                {
                    for (size_t i = 0; i < m_data.size(); ++i)
                    {
                        m_stats.m_sdev += pow(m_data[i] - m_stats.m_mean, 2);
                    }
                    m_stats.m_sdev = sqrt(m_stats.m_sdev / (m_data.size() - 1));
                }
            }
        }
    };

    DataSet&    getDataSet(const std::string& name)
    {
        auto entry = m_lookup.find(name);
        if (entry != m_lookup.end())
        {
            return m_dataSets[entry->second];
        }
        m_lookup[name] = m_dataSets.size();
        m_dataSets.push_back(DataSet());
        return m_dataSets.back();
    }

    bool        dataSetExists(const std::string& name) const
    {
        return m_lookup.find(name) != m_lookup.end();
    }

    void        calculateStats()
    {
        for (size_t i = 0; i < m_dataSets.size(); ++i)
        {
            m_dataSets[i].calculateStats();
        }
    }

    void        test(DataCollection<int64_t>& calibration, double relativeThreshold = 0.10, double tickThreshold = 100.0)
    {
        for (auto entry = m_lookup.begin(); entry != m_lookup.end(); ++entry)
        {
            const std::string& name = entry->first;
            DataCollection<int64_t>::DataSet& data = m_dataSets[entry->second];
            data.calculateStats();

            if (!calibration.dataSetExists(name))
            {
                FAIL() << "PerfTest is not calibrated!" << std::endl << "Missing DataSet: " << name << std::endl;
            }
            const DataCollection<int64_t>::DataSet& cal = calibration.getDataSet(name);
            const double calMin = cal.m_stats.m_min;
            
            if (data.m_stats.m_min > (1.0 + relativeThreshold) * calMin && data.m_stats.m_min - calMin > tickThreshold)
            {
                std::cout << name << ":" << std::endl;
                std::cout << "PERF - : Timing (" << data.m_stats.m_min << ") exceeds recorded min (" << calMin << ") by more than allowed relative threshold (" << relativeThreshold*100 << "%) and absolute threshold (" << tickThreshold << " ticks)." << std::endl;
                EXPECT_FALSE(data.m_stats.m_min > (1.0 + relativeThreshold) * calMin && data.m_stats.m_min - calMin > tickThreshold) 
                    << name << ":" << std::endl
                    << "PERF - : Timing (" << data.m_stats.m_min << ") exceeds recorded min (" << calMin << ") by more than allowed relative threshold (" << relativeThreshold * 100 << "%) and absolute threshold (" << tickThreshold << " ticks)." << std::endl;
            }
            else
            if (data.m_stats.m_min < (1.0 - relativeThreshold) * calMin && data.m_stats.m_min - calMin < -tickThreshold)
            {
                std::cout << name << ":" << std::endl;
                std::cout << "PERF + : Timing (" << data.m_stats.m_min << ") is less than the recorded min (" << calMin << ") by more than the relative threshold (" << relativeThreshold * 100 << "%) and absolute threshold (" << tickThreshold << " ticks)." << std::endl;
            }
        }
    }

    size_t      size() const
    {
        return m_dataSets.size();
    }

    void        clear()
    {
        m_lookup.clear();
        m_dataSets.clear();
    }

    template<class S>
    friend std::istream&    operator >> (std::istream& stream, DataCollection<S>& c);

    template<class S>
    friend std::ostream&    operator << (std::ostream& stream, const DataCollection<S>& c);

private:
    std::map<std::string, size_t>   m_lookup;
    std::vector< DataSet >          m_dataSets;
};

template<typename T>
std::istream&   operator >> (std::istream& stream, DataCollection<T>& c)
{
    std::string name;
    while (!stream.eof())
    {
        std::getline(stream >> std::ws, name);
        typename DataCollection<T>::DataSet& dataSet = c.getDataSet(name);
        stream >> dataSet.m_stats.m_mean >> dataSet.m_stats.m_sdev >> dataSet.m_stats.m_min >> dataSet.m_stats.m_max >> std::ws;
    }
    return stream;
}

template<typename T>
std::ostream&   operator << (std::ostream& stream, const DataCollection<T>& c)
{
    for (auto entry = c.m_lookup.begin(); entry != c.m_lookup.end(); ++entry)
    {
        const std::string& name = entry->first;
        stream << name.c_str() << std::endl;
        const typename DataCollection<T>::DataSet& data = c.m_dataSets[entry->second];
        stream << data.m_stats.m_mean << " " << data.m_stats.m_sdev << " " << data.m_stats.m_min << " " << data.m_stats.m_max << std::endl;
    }
    return stream;
}


static const char* getPlatformSuffix()
{
#if NV_WIN32
    return "win32";
#elif NV_WIN64
    return "win64";
#elif NV_LINUX 
    #if NV_X64
        return "linux64";
    #else 
        return "linux32";
    #endif
#else
    return "gen";
#endif
}

static const char* getPlatformRoot()
{
#if NV_LINUX
    return "../../";
#else
    return "../../../";
#endif
}

static std::string defaultRelativeDataPath()
{
    const char* dataDir = "test/data/";

    std::string rootDir = getPlatformRoot();
    return rootDir + dataDir + getPlatformSuffix() + "/";
}

class PerfTestEngine
{
public:
    PerfTestEngine(const char* collectionName) : m_calibrate(false)
    {
        m_filename = defaultRelativeDataPath() + std::string(collectionName) + "_" + getPlatformSuffix() + ".cal";

        auto argvs = testing::internal::GetArgvs();
        size_t argCount = argvs.size();
            
        for (size_t argNum = 0; argNum < argCount; ++argNum)
        {
            if (argvs[argNum] == "-calibrate")
            {
                m_calibrate = true;
            }
            else
            if (argvs[argNum] == "-calPath")
            {
                if (++argNum < argCount)
                {
                    m_filename = argvs[argNum];
                }
            }
        }

        if (!m_calibrate)
        {
            std::ifstream in;
            in.open(m_filename);
            if (in.is_open())
            {
                std::string name;
                std::getline(in, name); // Eat header
                std::getline(in, name); // Eat header (2 lines)
                in >> m_dataCalibration;
                in.close();
            }
            m_calibrate = m_dataCalibration.size() == 0;
        }

        if (m_calibrate)
        {
            std::ofstream out;
            out.open(m_filename);
            if (out.is_open())
            {
                out << "Format: timing name (whole line)" << std::endl << "timing mean s.d. min  max" << std::endl; // Header (2 lines)
                out.close();
            }
        }

        if (m_calibrate)
        {
            std::cout << "******** Calibration Mode ********\n";
        }
        else
        {
            std::cout << "******** Test Mode ********\n";
            std::cout << "Read calibration data from " << m_filename << std::endl;
        }
    }

    void    endTest()
    {
        if (m_calibrate)
        {
            m_dataTempCollection.calculateStats();
            std::ofstream out;
            out.open(m_filename, std::ofstream::app);
            if (out.is_open())
            {
                out << m_dataTempCollection;
                out.close();
                std::cout << "Calibration stats written to " << m_filename << std::endl;
            }
            else
            {
                std::cout << "Failed to open calibration file " << m_filename << ".  Stats not written." << std::endl;
                FAIL() << "Failed to open calibration file " << m_filename << ".  Stats not written." << std::endl;
            }
        }
        else
        {
            m_dataTempCollection.test(m_dataCalibration);
        }
        m_dataTempCollection.clear();
    }

    void    reportData(const std::string& name, int64_t data)
    {
        m_dataTempCollection.getDataSet(name).m_data.push_back(data);
    }

private:
    std::string             m_filename;
    bool                    m_calibrate;
    DataCollection<int64_t> m_dataTempCollection;
    DataCollection<int64_t> m_dataCalibration;
};


template<int FailLevel, int Verbosity>
class BlastBasePerfTest : public BlastBaseTest<FailLevel, Verbosity>
{
public:
    /**
    This function allows to create/destroy and get PerfTestEngine in local static variable (works header only).
    It allows to have PeftTestEngine alive through life span of gtest TestCase.
    */
    static PerfTestEngine* getEngineDeadOrAlive(bool alive = true)
    {
        static PerfTestEngine* engine = nullptr;
        if (alive && !engine)
        {
            engine = new PerfTestEngine(::testing::UnitTest::GetInstance()->current_test_case()->name());
        }
        else if (!alive && engine)
        {
            delete engine;
            engine = nullptr;
        }
        return engine;
    }

    static void SetUpTestCase()
    {
        getEngineDeadOrAlive();
    }

    static void TearDownTestCase()
    {
        getEngineDeadOrAlive(false);
    }

    void        TearDown() override
    {
        getEngineDeadOrAlive()->endTest();
    }

    void        reportData(const std::string& name, int64_t data)
    {
        getEngineDeadOrAlive()->reportData(name, data);
    }
};


#endif // #ifndef BLASTBASEPERFTEST_H
