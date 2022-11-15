#ifndef PARAMSUTILS_H
#define PARAMSUTILS_H

#include "yaml-cpp/yaml.h"
#include "fstream"

namespace Configuration
{


struct GraphParams
{
    std::string tss_maindir;
    std::string tss_filename;
    std::string bag_filename;
    std::string WO_topic;
    std::string GPS_topic;
    std::string VO_topic;
    std::string SVO_topic;
    std::string GT_topic;
    std::string JAI_RGB_topic;
    std::string IMU_topic;
    std::string CLOUD_topic;
    
    std::string GPS_calib;
    std::string VO_calib;
    std::string SVO_calib;
    std::string GT_calib;

    bool enable[4];
    bool enable_MA;
    bool enable_elevation;
    bool enable_ackerman;
    bool enable_imu;
    bool enable_velodyne;

    double WO_step;
    double GPS_step;
    double VO_step;
    double SVO_step;
    double CLOUD_step;

    double WO_scale_t;
    double WO_scale_r;
    double GPS_scale_txy;
    double GPS_scale_tz;
    double VO_scale_t;
    double VO_scale_r;
    double SVO_scale_t;
    double SVO_scale_tz;
    double SVO_scale_r;
    double ACKERMAN_scale_t;
    double ACKERMAN_scale_r;
    double IMU_scale;
    double CLOUD_scale;
    
    double MA_radius;
    double MA_scale;
    
    int sw_size;
    double sw_scale;
    
    double elevation_scale;

    std::string graph_filename;
    
    std::string elevation_map_file;
    
    std::string out_folder;

    void writeParams(const std::string yaml_filename)
    {
        YAML::Node params;

        params["tss_maindir"]=tss_maindir;
        params["tss_filename"]=tss_filename;
        params["bag_filename"]=bag_filename;
        params["WO_topic"]=WO_topic;
        params["GPS_topic"]=GPS_topic;
        params["VO_topic"]=VO_topic;
        params["SVO_topic"]=SVO_topic;
        params["GT_topic"]=GT_topic;
        params["JAI_RGB_topic"]=JAI_RGB_topic;
        params["IMU_topic"]=IMU_topic;
        params["CLOUD_topic"]=CLOUD_topic;
        
        params["GPS_calib"]=GPS_calib;
        params["VO_calib"]=VO_calib;
        params["SVO_calib"]=SVO_calib;
        params["GT_calib"]=GT_calib;

        params["enable"].push_back(enable[0]);
        params["enable"].push_back(enable[1]);
        params["enable"].push_back(enable[2]);
        params["enable"].push_back(enable[3]);
        
        params["enable_MA"]=enable_MA;
        params["enable_elevation"]=enable_elevation;
        params["enable_ackerman"]=enable_ackerman;
        params["enable_imu"]=enable_imu;
        params["enable_velodyne"]=enable_velodyne;

        params["WO_step"]=WO_step;
        params["GPS_step"]=GPS_step;
        params["VO_step"]=VO_step;
        params["SVO_step"]=SVO_step;
        params["CLOUD_step"]=CLOUD_step;

        params["WO_scale_t"]=WO_scale_t;
        params["WO_scale_r"]=WO_scale_r;
        params["GPS_scale_txy"]=GPS_scale_txy;
        params["GPS_scale_tz"]=GPS_scale_tz;
        params["VO_scale_t"]=VO_scale_t;
        params["VO_scale_r"]=VO_scale_r;
        params["SVO_scale_t"]=SVO_scale_t;
        params["SVO_scale_tz"]=SVO_scale_tz;
        params["SVO_scale_r"]=SVO_scale_r;
        
        params["ACKERMAN_scale_t"]=ACKERMAN_scale_t;
        params["ACKERMAN_scale_r"]=ACKERMAN_scale_r;
        
        params["IMU_scale"]=IMU_scale;
        params["CLOUD_scale"]=CLOUD_scale;
        
        params["MA_radius"]=MA_radius;
        params["MA_scale"]=MA_scale;
        
        params["sw_scale"]=sw_scale;
        params["sw_size"]=sw_size;
        
        params["elevation_scale"]=elevation_scale;

        params["graph_filename"]=graph_filename;
        
        params["elevation_map_file"]=elevation_map_file;
        
        params["out_folder"]=out_folder;

        std::ofstream fout(yaml_filename.c_str());
        fout<<params;
        fout.close();
    }

    void readParams(const std::string yaml_filename)
    {
        YAML::Node params = YAML::LoadFile(yaml_filename);

        tss_maindir=params["tss_maindir"].as<std::string>();
        tss_filename=params["tss_filename"].as<std::string>();
        bag_filename=params["bag_filename"].as<std::string>();
        WO_topic=params["WO_topic"].as<std::string>();
        GPS_topic=params["GPS_topic"].as<std::string>();
        VO_topic=params["VO_topic"].as<std::string>();
        SVO_topic=params["SVO_topic"].as<std::string>();
        GT_topic=params["GT_topic"].as<std::string>();
        JAI_RGB_topic=params["JAI_RGB_topic"].as<std::string>();
        IMU_topic=params["IMU_topic"].as<std::string>();
        CLOUD_topic=params["CLOUD_topic"].as<std::string>();
        
        GPS_calib=params["GPS_calib"].as<std::string>();
        VO_calib=params["VO_calib"].as<std::string>();
        SVO_calib=params["SVO_calib"].as<std::string>();
        GT_calib=params["GT_calib"].as<std::string>();

        for(int i=0; i<4; i++)
            enable[i]=params["enable"][i].as<bool>();
        
        enable_MA=params["enable_MA"].as<bool>();
        enable_elevation=params["enable_elevation"].as<bool>();
        enable_ackerman=params["enable_ackerman"].as<bool>();
        enable_imu=params["enable_imu"].as<bool>();
        enable_velodyne=params["enable_velodyne"].as<bool>();

        WO_step=params["WO_step"].as<double>();
        GPS_step=params["GPS_step"].as<double>();
        VO_step=params["VO_step"].as<double>();
        SVO_step=params["SVO_step"].as<double>();
        CLOUD_step=params["CLOUD_step"].as<double>();

        WO_scale_t=params["WO_scale_t"].as<double>();
        WO_scale_r=params["WO_scale_r"].as<double>();
        GPS_scale_txy=params["GPS_scale_txy"].as<double>();
        GPS_scale_tz=params["GPS_scale_tz"].as<double>();
        VO_scale_t=params["VO_scale_t"].as<double>();
        VO_scale_r=params["VO_scale_r"].as<double>();
        SVO_scale_t=params["SVO_scale_t"].as<double>();
        SVO_scale_tz=params["SVO_scale_tz"].as<double>();
        SVO_scale_r=params["SVO_scale_r"].as<double>();
        
        ACKERMAN_scale_t=params["ACKERMAN_scale_t"].as<double>();
        ACKERMAN_scale_r=params["ACKERMAN_scale_r"].as<double>();
        
        IMU_scale=params["IMU_scale"].as<double>();
        CLOUD_scale=params["CLOUD_scale"].as<double>();
        
        MA_radius=params["MA_radius"].as<double>();
        MA_scale=params["MA_scale"].as<double>();
        
        sw_scale=params["sw_scale"].as<double>();
        sw_size=params["sw_size"].as<int>();
        
        elevation_scale=params["elevation_scale"].as<double>();

        graph_filename=params["graph_filename"].as<std::string>();
        
        elevation_map_file=params["elevation_map_file"].as<std::string>();
        
        out_folder=params["out_folder"].as<std::string>();
    }

};

struct OptimizerParams
{
    int N_iterations;
    std::string in_filename;
    std::string out_filename;

    void writeParams(const std::string yaml_filename)
    {
        YAML::Node params;
        params["N_iterations"]=N_iterations;
        params["in_filename"]=in_filename;
        params["out_filename"]=out_filename;

        std::ofstream fout(yaml_filename.c_str());
        fout<<params;
        fout.close();
    }

    void readParams(const std::string yaml_filename)
    {
        YAML::Node params = YAML::LoadFile(yaml_filename);
        N_iterations=params["N_iterations"].as<int>();
        in_filename=params["in_filename"].as<std::string>();
        out_filename=params["out_filename"].as<std::string>();
    }
};

}

#endif // PARAMSUTILS_H

