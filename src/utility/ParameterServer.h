//
// Created by xiongyi on 3/28/16.
//

#ifndef CXY_LSDSLAM_PARAMETERSERVER_H
#define CXY_LSDSLAM_PARAMETERSERVER_H
//
// Created by xiongyi on 3/28/16.
//
#include <yaml-cpp/yaml.h>

namespace cxy{
    class ParameterServer
    {
    private:
        YAML::Node mConfig;

        ParameterServer();
        static ParameterServer* instance;
    public:
        static ParameterServer* getInstance();

        template <typename T>
        static T getParameter(const std::string &paraName);

        static YAML::Node getParameterNode(const std::string &paraName);

    };
}


#endif //CXY_LSDSLAM_PARAMETERSERVER_H
