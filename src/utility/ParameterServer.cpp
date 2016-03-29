//
// Created by xiongyi on 3/28/16.
//
#include "ParameterServer.h"

cxy::ParameterServer* cxy::ParameterServer::instance = nullptr;

namespace cxy{
    ParameterServer::ParameterServer()
    {
        std::string yamlConfig = "/home/xiongyi/workspace/src/cxy-LSD-SLAM/cfg/config.yaml";
        mConfig = YAML::LoadFile(yamlConfig);
    }
     ParameterServer* ParameterServer::getInstance()
    {

        if (nullptr == instance)
            instance = new ParameterServer();
        return instance;
    }

    template <typename T>
    T ParameterServer::getParameter(const std::string &paraName)
    {
        return getInstance()->mConfig[paraName].as<T>();
    }
}

