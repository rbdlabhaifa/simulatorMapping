#include <opencv2/core/core.hpp>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include "DBoW3/src/Vocabulary.h"
#include"DBoW3/src/DescManip.h"
#include <memory>
// this file DOES NOT give proper weights to leaf nodes, this is done by python
// expecting argv[0] := branching factor, argv[1] := depth, argv[2] := new tree file name
int main(int argc, char **argv)
{

    if(argc < 5)
    {
        std::cout << "Usage: ./create_trees <leaves_path> <branching_factor> <depth> <effective_depth> <saved_tree_file_name>"  << std::endl;
        return 1;
    }

    const std::string leaves_path = std::string(argv[1]);
    int branching_factor = atoi(argv[2]);
    int depth = atoi(argv[3]);
    int effective_depth = atoi(argv[4]);
    const std::string saved_tree_file_name = std::string(argv[5]);

    auto mpVocabulary = std::make_shared<DBoW3::Vocabulary>(branching_factor, depth, DBoW3::WeightingType::TF_IDF, DBoW3::ScoringType::DOT_PRODUCT);

    std::cout << "gathering leaves from " << leaves_path << "..." << std::endl;

    std::ifstream f(leaves_path);
    std::string s;
    std::vector<cv::Mat> descriptors;

    while (std::getline(f, s))
    {
        cv::Mat p;
        DBoW3::DescManip::fromString(p, s);
        descriptors.push_back(p);
    }

    std::cout << "creating vocabulary tree with: branching factor=" << branching_factor << ", maximal depth=" << effective_depth << "..." << std::endl;
    mpVocabulary->create(descriptors);

    std::cout << "tree created successfully! saving temporary tree to:" << saved_tree_file_name << "..." << std::endl;
    mpVocabulary->save(saved_tree_file_name,false);
    std::cout << "temporary tree saved!" << std::endl;

    return 0;
}