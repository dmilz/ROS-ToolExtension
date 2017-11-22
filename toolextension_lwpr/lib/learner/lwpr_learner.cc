#include "lwpr_learner.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <vector>

namespace toolextension_lwpr
{

    // For printing vectors
    template<typename T>
    std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
    {
       using namespace std;
       os<<"[ ";
       copy(v.begin(), v.end(), ostream_iterator<T>(os, " "));
       os<<"]";
       return os;
    }

    LWPR_Learner::LWPR_Learner(Config& config, std::string prefix)
            : BaseLearner(LWPR_Learner::name(), config, prefix)
    {
            // optionally reload model
            if(config.getBool(this->config_prefix + "reload_model", false))
            {
                INFO("Reload model from " << config.getString(this->config_prefix + "model_folder","~"));
                read(config.getString(this->config_prefix + "path","~"));
            }
            else
            {
                // common parameters
                int actionDim   = config.getInt(prefix+"actionDim", 1);
                int stateDim    = config.getInt(prefix+"stateDim", 1);

                // lwpr parameters
                cutoff          = config.getDouble(this->config_prefix + "cutoff", 0.001);
                max_wMax        = config.getDouble(this->config_prefix + "max_wMax", 0.2);

                // create new model
                model           = new LWPR_Object(stateDim,actionDim);

                updateModel();
            }
    }

    LWPR_Learner::~LWPR_Learner()
    {
        if(config.getBool(this->config_prefix + "store_model",false))
        {
            const std::string path = config.getString(this->config_prefix + "path","");
            if(boost::filesystem::exists(boost::filesystem::path(path).parent_path().string()))
            {
                INFO("Deleting LWPR_Learner and storing at \"" << path << "\"");
                this->write(path);
            }
            else
            {
                INFO("No such file \"" << path << "\"");
            }
        }
        else
        {
            INFO("Deleting LWPR_Learner");
        }
        delete model;
    }

    void LWPR_Learner::updateModel()
    {
            int actionDim   = config.getInt(this->config_prefix + "actionDim", 1);
            int stateDim    = config.getInt(this->config_prefix + "stateDim", 1);

            // normalization
            if(upperInputBounds.size() == stateDim && upperOutputBounds.size() == actionDim)
            {
                    model->normIn(upperInputBounds);
                    model->normOut(upperOutputBounds);
            }
            else
            {
                    std::cout << "Invalid normalization." << std::endl;
            }

            // #### distance metric
            model->setInitAlpha(config.getDouble(this->config_prefix + "initAlpha", 100));

            // meta learning rate
            model->useMeta(config.getBool(this->config_prefix + "useMeta", false));
            model->metaRate(config.getDouble(this->config_prefix + "metaRate", 100));

            // penalty factor (higher -> smoother) default: 1e-6
            model->penalty(config.getDouble(this->config_prefix + "penalty", 1e-6));

            /* Set initial distance metric to 50*(identity matrix) */
            model->setInitD(config.getDouble(this->config_prefix + "initD", 50));
            model->updateD(config.getBool(this->config_prefix + "updateD", false));

            model->initLambda(config.getDouble(this->config_prefix + "initLambda", 0.999));
            model->finalLambda(config.getDouble(this->config_prefix + "finalLambda", 0.99999));
            model->tauLambda(config.getDouble(this->config_prefix + "tauLambda", 0.9999));


            // #### local regression
            model->wGen(config.getDouble(this->config_prefix + "wGen", 0.1));
    }


    Vector LWPR_Learner::train(const Vector& in, const Vector& out )
    {
            m_mutex.lock();
            Vector ypre;
            try
            {
                    ypre = (Vector) model->update((doubleVec) in, (doubleVec) out);
                    this->nData++;
                    //Calc MSE
                    double sum=0;
                    for(int i = 0; i < ypre.size(); i++)
                        sum += (ypre[i]-out[i]);
                    this->_mse = (this->_mse*(this->nData-1)+sum)/this->nData;
            }
            catch(LWPR_Exception& e)
            {
                    std::cout << "In: " << in << "\n Out:" << out << std::endl;
                    std::cerr << "addData: LWPR error: " << e.getString() << std::endl;
            }
            m_mutex.unlock();
            return ypre;
    }

    Vector LWPR_Learner::predict(const Vector& in, const Vector& tolerance)
    {
            doubleVec yp,conf,wMax;

            m_mutex.lock();
            try
            {
                    yp = model->predict(in, conf, wMax, cutoff);
            }
            catch(LWPR_Exception& e)
            {
                    std::cerr << "getActionComp: LWPR error: " << e.getString() << " stateSize:"
                                    << in.size() << " modelInDim:" << model->nIn() << std::endl;
            }
            m_mutex.unlock();

            for(int i=0;i<yp.size();i++)
            {
                    if(model->nData() < min_nData) //if((wMax[i] < max_wMax) || (model->nData() < min_nData))
                    {
                            yp[i] = 0;
                    }
                    // cut off at bounds
                    yp[i] = fminf(upperOutputBounds[i] , yp[i]);
                    yp[i] = fmaxf(-upperOutputBounds[i], yp[i]);
            }

           // tolerance = conf;

            return (Vector) yp;
    }

    void LWPR_Learner::read(const std::string& file_wo_suffix)
    {
        std::string file = file_wo_suffix + ".bin";
        INFO("Reading \"" << file << "\"");
        if(boost::filesystem::exists(file))
        {
                model = new LWPR_Object(file.c_str());
                INFO("Reusing existing model with " << model->nData() << " nData." << std::endl);
                updateModel();
        }
        else
        {
                INFO("Model could not be reloaded! Default initialization.");
                int actionDim   = config.getInt(config_prefix+"actionDim", 1);
                int stateDim    = config.getInt(config_prefix+"stateDim", 1);
                model           = new LWPR_Object(stateDim,actionDim);
        }
    }

    void LWPR_Learner::write(const std::string& file_wo_suffix)
    {
        INFO("Save LWPR model with " << model->nData() << " nData to " << file_wo_suffix << std::endl);
        m_mutex.lock();
        std::string binfile = file_wo_suffix + ".bin";
        std::string xmlfile = file_wo_suffix + ".xml";
        model->writeBinary(binfile.c_str());
        model->writeXML(xmlfile.c_str());
        m_mutex.unlock();
    }

}
