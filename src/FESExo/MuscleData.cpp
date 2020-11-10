#include <FESExo/MuscleData.hpp>

MuscleData::MuscleData(std::vector<MuscleInfo> muscle_infos):
m_muscle_infos(muscle_infos)
{
    for (const auto &muscle : m_muscle_infos){
        m_amplitudes.push_back(muscle.amplitude);
        m_min_pulsewidths.push_back(muscle.min_pulsewidth);
        m_max_pulsewidths.push_back(muscle.max_pulsewidth);
        m_muscle_names.push_back(muscle.muscle_name);
    }
}

MuscleData::MuscleData()
{
    // create one default muscle_infos and call non-default constructor
    std::vector<MuscleInfo> muscle_infos;
    muscle_infos.emplace_back();

    for (const auto &muscle : m_muscle_infos){
        m_amplitudes.push_back(muscle.amplitude);
        m_min_pulsewidths.push_back(muscle.min_pulsewidth);
        m_max_pulsewidths.push_back(muscle.max_pulsewidth);
        m_muscle_names.push_back(muscle.muscle_name);
    }
}

void to_json(nlohmann::json& j, const MuscleInfo& p) {
    j = nlohmann::json{{"muscle_num", p.muscle_num}, {"muscle_name", p.muscle_name}, {"amplitude", p.amplitude}, {"min_pulsewidth", p.min_pulsewidth}, {"max_pulsewidth", p.max_pulsewidth}};
}

void from_json(const nlohmann::json& j, MuscleInfo& p) {
    j.at("muscle_num").get_to(p.muscle_num);
    j.at("muscle_name").get_to(p.muscle_name);
    j.at("amplitude").get_to(p.amplitude);
    j.at("min_pulsewidth").get_to(p.min_pulsewidth);
    j.at("max_pulsewidth").get_to(p.max_pulsewidth);
}