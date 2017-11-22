#ifndef LWPRLEARNER_H_
#define LWPRLEARNER_H_

#include "base_learner.h"
#include <string>
#include <vector>
#include <lwpr.hh>

#define LWPRLEARNER_VERBOSE
#ifdef LWPRLEARNER_VERBOSE
#include <iostream>
#define  INFO(x)  std::cout<<"[ INFO] [LWPR Learner]: "<<x<<std::endl
#else
#define INFO(x)
#endif


namespace toolextension_lwpr
{
    class LWPR_Learner : public BaseLearner
    {
    public:
        LWPR_Learner(Config& config, std::string prefix = "");
        virtual ~LWPR_Learner();
        Vector train  (const Vector& in, const Vector& out );
        Vector predict(const Vector& in, const Vector& tolerance);
        void read(const std::string& file_wo_suffix);
        void write(const std::string& file_wo_suffix);
        static std::string name() { return "lwpr"; }
        std::string getName(){ return this->config_prefix; }
    private:
        void updateModel();

        LWPR_Object* model;
        double cutoff;
        double max_wMax;
    };

}

#endif
