//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

/**
 * Program to convert Poikonnen instances as obtained to json format
 */

#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>

#include "tspd_instance.h"

using namespace std;
using namespace tspd;
namespace fs = filesystem;

struct IndexEntry{
    string instance_name;
    string file_name;
    vector<string> tags;
    size_t vertex_count;
};

void to_json(nlohmann::json& j, const IndexEntry& index) 
{
    j["instance_name"] = index.instance_name;
    j["file_name"] = index.file_name;
    j["tags"] = index.tags;
}

using Index = vector<IndexEntry>;

int main(int argc, char** argv)
{
    //{ Read input
    if(argc < 3) {
        cerr << "Usage instance_converter <indir> <outdir> \n\n"
             << "where <indir> contains the poikonnen instances to convert.\n"
             << "      [outdir] is the directory to put the JSON instances.\n ";
        exit(1);
    }

    cerr << "Reading from path " << argv[1] << endl;
    cerr << "Writing to path " << argv[2] << endl;
    
    const fs::path inpath(argv[1]);
    const fs::path outdir(argv[2]);
    
    for(auto& p : {inpath, outdir}) if(not fs::exists(p) or not fs::is_directory(p)) {
        cerr << "Path " << fs::current_path() << " " << p << " does not exist or is not a directory" << endl;
        exit(2);
    }    
    
    vector<IndexEntry> index;
    for (auto& entry : fs::directory_iterator(inpath)) 
    if(entry.is_regular_file() and entry.path().extension() == ".txt") {
        cerr << "Reading file " << entry.path() << endl;
        std::ifstream is(entry.path());        
        TSPDInput tsp;
        from_poi(is, tsp);
                
        nlohmann::json json;
        to_json(json, tsp);
        
        auto outpath = outdir / entry.path().stem();
        outpath += ".json";
        cerr << "Output to file " << outpath << endl;

        IndexEntry index_entry;
        index_entry.instance_name = entry.path().stem();
        index_entry.file_name = index_entry.instance_name + ".json";
        index_entry.tags.push_back(index_entry.instance_name);
        index_entry.tags.push_back(string("N") + STR(tsp.vertex_count-1));
        index_entry.vertex_count = tsp.vertex_count-1;
        index.push_back(index_entry);

        ofstream out(outpath);
        out << setw(4) << json;
        out.close();
    }
    
    sort(index.begin(), index.end(), [](auto i, auto j) {
        return i.vertex_count < j.vertex_count or (i.vertex_count == j.vertex_count and i.file_name < j.file_name);
    });
    nlohmann::json index_json;
    to_json(index_json, index);
    
    ofstream out(outdir / "index.json");
    out << setw(4) << index_json;
    out.close();
        
    return 0;
}
