#include <Config.h>
#include <iostream>

bool Config::instanceFlag = false;
bool Config::isParsed = false;
int Config::m_version;
Config* Config::single = NULL;
Config* Config::instance()
{
    if(! instanceFlag)
    {
        single = new Config();
        instanceFlag = true;
    }
    if(!isParsed)
    {
        std::cout<<"Parsing failed!\n";
    }
    return single;
}
Config* Config::refresh()
{
    if(instanceFlag)
    {
        delete single;
    }
    single = new Config();
    instanceFlag = true;
    if(!isParsed)
    {
        std::cout<<"Parsing failed!\n";
    }
    return single;
}

int Config::GetVersion()
{
    return m_version;
}

void Config::SetVersion(int version)
{
    m_version = version;
}

bool Config::valueToVector(Json::Value& value,std::vector<double>& vector){
  if(value.isArray()){
    vector.resize(value.size());
    for(size_t i=0;i<value.size();i++){
      vector[i] = value[i].asDouble();
    }
    return true;
  }
  return false;
}
