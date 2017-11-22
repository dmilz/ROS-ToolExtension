#ifndef  BaseLearner_H_
#define  BaseLearner_H_

#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>

#include "config.h"

namespace toolextension_lwpr
{

    typedef std::vector<double> Vector;
    enum STATE { training = 0, predicting = 1, both = 2 };


    class BaseLearner
    {
    public:
        BaseLearner(std::string name, Config& config, std::string prefix = "");
        virtual ~BaseLearner();
        virtual Vector train  (const Vector& in, const Vector& out ) = 0;
        virtual Vector predict(const Vector& in, const Vector& tolerance) = 0;
        virtual void read(const std::string& file_wo_suffix) = 0;
        virtual void write(const std::string& file_wo_suffix) = 0;

        Config& getConfig();

        std::string getName();
        int getNData();
        double getMSE();

    private:
        std::string name;

    protected:
        Vector upperInputBounds;
        Vector upperOutputBounds;
        Config config;
        boost::mutex m_mutex;
        std::string config_prefix;
        int min_nData;
        int nData;
        STATE state;
        double _mse;
    };

} /* namespace */

#endif
