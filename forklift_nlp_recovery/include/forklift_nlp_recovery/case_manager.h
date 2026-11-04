#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sandbox_msgs/RecoverySnapshot.h>

namespace forklift_nlp_recovery {

struct CaseDirectory {
    bool success = false;
    unsigned int number = 0;
    std::string path;
    std::string reason;
};

struct ResultDirectory {
    bool success = false;
    unsigned int case_number = 0;
    unsigned int run_number = 0;
    std::string path;
    std::string reason;
};

class CaseManager {
public:
    CaseManager(std::string case_root, std::string results_root);

    CaseDirectory createCase(const sandbox_msgs::RecoverySnapshot& snapshot,
                             const std::vector<int>& participant_ids) const;
    ResultDirectory createResult(unsigned int case_number) const;
    bool writeRunStatus(const ResultDirectory& directory, const std::string& stage,
                        bool complete, bool success, const std::string& reason,
                        std::string& error) const;

    const std::string& caseRoot() const { return case_root_; }
    const std::string& resultsRoot() const { return results_root_; }

private:
    std::string case_root_;
    std::string results_root_;
};

bool loadRecoveryCase(const std::string& case_directory,
                      sandbox_msgs::RecoverySnapshot& snapshot,
                      std::vector<int>& participant_ids, std::string& error);

}  // namespace forklift_nlp_recovery
