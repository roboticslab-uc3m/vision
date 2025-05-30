/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Thu Apr 17 19:21:21 2025


#include "DnnDetector_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(DnnDetectorParamsCOMPONENT, "yarp.device.DnnDetector")
}


DnnDetector_ParamsParser::DnnDetector_ParamsParser()
{
}


std::vector<std::string> DnnDetector_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("trainedModel");
    params.push_back("configDNNModel");
    params.push_back("framework");
    params.push_back("classesTrainedModel");
    params.push_back("backend");
    params.push_back("target");
    params.push_back("scale");
    params.push_back("mean");
    params.push_back("confThreshold");
    params.push_back("nmsThreshold");
    return params;
}


bool      DnnDetector_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(DnnDetectorParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter trainedModel
    {
        if (config.check("trainedModel"))
        {
            m_trainedModel = config.find("trainedModel").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'trainedModel' using value:" << m_trainedModel;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'trainedModel' using DEFAULT value:" << m_trainedModel;
        }
        prop_check.unput("trainedModel");
    }

    //Parser of parameter configDNNModel
    {
        if (config.check("configDNNModel"))
        {
            m_configDNNModel = config.find("configDNNModel").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'configDNNModel' using value:" << m_configDNNModel;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'configDNNModel' using DEFAULT value:" << m_configDNNModel;
        }
        prop_check.unput("configDNNModel");
    }

    //Parser of parameter framework
    {
        if (config.check("framework"))
        {
            m_framework = config.find("framework").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'framework' using value:" << m_framework;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'framework' using DEFAULT value:" << m_framework;
        }
        prop_check.unput("framework");
    }

    //Parser of parameter classesTrainedModel
    {
        if (config.check("classesTrainedModel"))
        {
            m_classesTrainedModel = config.find("classesTrainedModel").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'classesTrainedModel' using value:" << m_classesTrainedModel;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'classesTrainedModel' using DEFAULT value:" << m_classesTrainedModel;
        }
        prop_check.unput("classesTrainedModel");
    }

    //Parser of parameter backend
    {
        if (config.check("backend"))
        {
            m_backend = config.find("backend").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'backend' using value:" << m_backend;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'backend' using DEFAULT value:" << m_backend;
        }
        prop_check.unput("backend");
    }

    //Parser of parameter target
    {
        if (config.check("target"))
        {
            m_target = config.find("target").asString();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'target' using value:" << m_target;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'target' using DEFAULT value:" << m_target;
        }
        prop_check.unput("target");
    }

    //Parser of parameter scale
    {
        if (config.check("scale"))
        {
            m_scale = config.find("scale").asFloat32();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'scale' using value:" << m_scale;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'scale' using DEFAULT value:" << m_scale;
        }
        prop_check.unput("scale");
    }

    //Parser of parameter mean
    {
        if (config.check("mean"))
        {
            m_mean = config.find("mean").asFloat64();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'mean' using value:" << m_mean;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'mean' using DEFAULT value:" << m_mean;
        }
        prop_check.unput("mean");
    }

    //Parser of parameter confThreshold
    {
        if (config.check("confThreshold"))
        {
            m_confThreshold = config.find("confThreshold").asFloat32();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'confThreshold' using value:" << m_confThreshold;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'confThreshold' using DEFAULT value:" << m_confThreshold;
        }
        prop_check.unput("confThreshold");
    }

    //Parser of parameter nmsThreshold
    {
        if (config.check("nmsThreshold"))
        {
            m_nmsThreshold = config.find("nmsThreshold").asFloat32();
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'nmsThreshold' using value:" << m_nmsThreshold;
        }
        else
        {
            yCInfo(DnnDetectorParamsCOMPONENT) << "Parameter 'nmsThreshold' using DEFAULT value:" << m_nmsThreshold;
        }
        prop_check.unput("nmsThreshold");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(DnnDetectorParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(DnnDetectorParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      DnnDetector_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: DnnDetector\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'trainedModel': trained model file\n");
    doc = doc + std::string("'configDNNModel': configuration file\n");
    doc = doc + std::string("'framework': framework\n");
    doc = doc + std::string("'classesTrainedModel': file with trained classes\n");
    doc = doc + std::string("'backend': backend\n");
    doc = doc + std::string("'target': target device\n");
    doc = doc + std::string("'scale': scale\n");
    doc = doc + std::string("'mean': mean\n");
    doc = doc + std::string("'confThreshold': confidence threshold\n");
    doc = doc + std::string("'nmsThreshold': non maximum suppression threshold\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device DnnDetector --trainedModel yolov3-tiny/yolov3-tiny.weights --configDNNModel yolov3-tiny/yolov3-tiny.cfg --framework darknet --classesTrainedModel coco-object-categories.txt --backend cuda --target cpu --scale 0.00392 --mean 0.0 --confThreshold 0.1 --nmsThreshold 0.4\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device DnnDetector\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
