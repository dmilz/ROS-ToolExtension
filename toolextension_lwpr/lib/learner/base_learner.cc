
#include "base_learner.h"
#include <iostream>

namespace toolextension_lwpr {


BaseLearner::BaseLearner(std::string name, Config& config, std::string prefix)
    : config_prefix(prefix), name(name), config(config)
{
        int actionDim = config.getInt(config_prefix+"actionDim", 1);
        int stateDim  = config.getInt(config_prefix+"stateDim", 1);
        min_nData     = config.getInt(config_prefix+"min_nData", 0);
        nData = 0;

        upperInputBounds = config.getDoubleVector(config_prefix+"upperInputBounds", upperInputBounds);
        if(upperInputBounds.empty())
        {
                std::cout << "No upper input bounds set!" << std::endl;
                for(int i=0;i<stateDim;i++) upperInputBounds.push_back(1);
        }
        else if(upperInputBounds.size() != stateDim)
        {
                std::cout << "Dimensions of upper input bounds do not match: " << upperInputBounds.size() << " != " << stateDim << std::endl;
                upperInputBounds.clear();
                for(int i=0;i<stateDim;i++) upperInputBounds.push_back(1);
        }

        upperOutputBounds = config.getDoubleVector(config_prefix+"upperOutputBounds", upperOutputBounds);
        if(upperOutputBounds.empty())
        {
                std::cout << "No upper output bounds set!" << std::endl;
                for(int i=0;i<actionDim;i++) upperOutputBounds.push_back(1);
        }
        else if(upperOutputBounds.size() != actionDim)
        {
                std::cout << "Dimensions of upper output bounds do not match: " << upperOutputBounds.size() << " != " << actionDim << std::endl;
                upperOutputBounds.clear();
                for(int i=0;i<actionDim;i++) upperOutputBounds.push_back(1);
        }
        this->_mse = 0;
}

BaseLearner::~BaseLearner()
{}

std::string BaseLearner::getName()
{
        return this->name;
}


Config& BaseLearner::getConfig()
{
        return this->config;
}

int BaseLearner::getNData()
{
    return this->nData;
}

double BaseLearner::getMSE()
{
    return this->_mse;
}

} /* namespace */
