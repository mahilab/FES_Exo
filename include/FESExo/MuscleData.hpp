#pragma once

#include <nlohmann/json.hpp>

struct MuscleInfo{
    MuscleInfo(unsigned int muscle_num_, std::string muscle_name_, unsigned int amplitude_, unsigned int min_pulsewidth_, unsigned int max_pulsewidth_) {
        muscle_num = muscle_num_;
        muscle_name = muscle_name_;
        amplitude = amplitude_;
        min_pulsewidth = min_pulsewidth_;
        max_pulsewidth = max_pulsewidth_;
    }
    MuscleInfo() {
        muscle_num     = 0;
        muscle_name    = "";
        amplitude      = 0;
        min_pulsewidth = 0;
        max_pulsewidth = 0;
    }
    unsigned int muscle_num;
    std::string muscle_name;
    unsigned int amplitude;
    unsigned int min_pulsewidth;
    unsigned int max_pulsewidth;
};

class MuscleData
{
private:
    std::vector<MuscleInfo>   m_muscle_infos;
    std::vector<unsigned int> m_amplitudes;
    std::vector<unsigned int> m_min_pulsewidths;
    std::vector<unsigned int> m_max_pulsewidths;
    std::vector<std::string>  m_muscle_names;
public:
    MuscleData(std::vector<MuscleInfo> muscle_infos);
    MuscleData();
    // ~MuscleData();
    std::vector<unsigned int> get_amplitudes(){return m_amplitudes;};
    std::vector<unsigned int> get_min_pulsewidths(){return m_min_pulsewidths;};
    std::vector<unsigned int> get_max_pulsewidths(){return m_max_pulsewidths;};
    std::vector<std::string> get_muscle_names(){return m_muscle_names;};
    std::vector<MuscleInfo> get_muscle_infos(){return m_muscle_infos;};

    unsigned int get_amplitude(int index){return m_amplitudes[index];};
    unsigned int get_min_pulsewidth(int index){return m_min_pulsewidths[index];};
    unsigned int get_max_pulsewidth(int index){return m_max_pulsewidths[index];};
    std::string get_muscle_name(int index){return m_muscle_names[index];};
    MuscleInfo get_muscle_info(int index){return m_muscle_infos[index];};
};

void to_json(nlohmann::json& j, const MuscleInfo& p);
void from_json(const nlohmann::json& j, MuscleInfo& p);