//	File "PrimateBFUGenBase.h"
//	
//	To perform backward and forward analysis of the source program
////////////////////////////////////////////////////////////////////////////////
#ifndef __PRIMATEBFUGENBASE_H__
#define __PRIMATEBFUGENBASE_H__

#include<llvm/Pass.h>
#include<llvm/IR/BasicBlock.h>
#include<llvm/IR/DebugInfo.h>
#include<llvm/Support/raw_ostream.h>
#include<llvm/IR/Instruction.h>
#include<llvm/IR/Instructions.h>
#include<llvm/IR/Constants.h>
#include<llvm/Transforms/Utils/BasicBlockUtils.h>
#include<llvm/ADT/BitVector.h>
#include<llvm/IR/ValueMap.h>
#include<llvm/IR/CFG.h>
#include<llvm/ADT/DenseMap.h>
#include "rapidjson/document.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include <math.h>
#include <map>
#include <tuple>
#include <string>

#include "PrimateBase.h"

using namespace llvm;

namespace {

class PrimateBFUGenBase : virtual public PrimateBase{
public:
    std::string filename;
    unsigned long long numTotalInst;
    unsigned long long numTotalExeCount;
    int numProfSegments;
    int **profSegments;
    int numProfRegions;
    int **profRegions;
    int numProfBranches;
    int **profBranches;
    int *numInstPerSeg;
    double avgExeCountPerInst;
    int numInst;
    int numLine;
    std::map<int, std::map<int, Value*>*> lineToBBs;
    ValueMap<Value*, std::string> varNameMap;
    ValueMap<Value*, int> varRegMap;
    // map<type name, vector<field name, field width, field type name>>
    std::map<std::string, std::vector<std::tuple<std::string, int, std::string>>*> varTypeMap;

    PrimateBFUGenBase() {}

    ~PrimateBFUGenBase(){
        delete [] numInstPerSeg;
        for (int i = 0; i < numProfSegments; i++) {
            delete [] profSegments[i];
        }
        delete [] profSegments;
        for (int i = 0; i < numProfRegions; i++) {
            delete [] profRegions[i];
        }
        delete [] profRegions;
        for (int i = 0; i < numProfBranches; i++) {
            delete [] profBranches[i];
        }
        delete [] profBranches;
    }

    void printBasicBlock(BasicBlock &B) {
        for (auto inst_it = B.begin(); inst_it != B.end(); inst_it++) {
            inst_it->print(errs());
            errs() << "\n";
        }
    }

    BasicBlock* getBBwithFunc(Function &F, const std::string func_name, int &line) {
        for (auto bb_it = F.begin(); bb_it != F.end(); bb_it++) {
            for (auto inst_it = bb_it->begin(); inst_it != bb_it->end(); inst_it++) {
                Instruction *inst = &*inst_it;
                if (isa<CallInst>(*inst)) {
                    auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                    Function* foo = tmp->getCalledFunction();
                    if (foo->getName().contains(func_name)) {
                        if (DILocation *Loc = inst->getDebugLoc()) {
                            line = Loc->getLine();
                        }
                        return &*bb_it;
                    }
                }
            }
        }
        return NULL;
    }

    void initializeVarNameMap(Function &F) {
        int i = 1;
        for (inst_iterator inst_it = inst_begin(F), e = inst_end(F); inst_it != e; ++inst_it) {
            if (isa<DbgDeclareInst>(*inst_it)) {
                auto* dbgInst = dyn_cast<DbgDeclareInst>(&*inst_it);
                auto* instMeta = cast<MetadataAsValue>(dbgInst->getOperand(0))->getMetadata();
                Value* inst = cast<ValueAsMetadata>(instMeta)->getValue();
                auto *varMeta = cast<DILocalVariable>(cast<MetadataAsValue>(dbgInst->getOperand(1))->getMetadata());
                std::string name = (varMeta->getName()).str();
                varNameMap[inst] = name;
                // errs() << "name: " << name << "\n";
                std::vector<int> fieldWidth;
                if (isa<AllocaInst>(*inst)) {
                    auto* aInst = dyn_cast<AllocaInst>(inst);
                    Type *aType = aInst->getAllocatedType();
                    if (isa<StructType>(*aType)) {
                        auto stype = dyn_cast<StructType>(aType);
                        for (auto elem = stype->element_begin(); elem != stype->element_end(); elem++) {
                            if (isa<IntegerType>(**elem)) {
                                unsigned elemWidth = (*elem)->getIntegerBitWidth();
                                fieldWidth.emplace_back(elemWidth);
                            } else if (isa<StructType>(**elem)) {
                                auto *struct_type = dyn_cast<StructType>(*elem);
                                unsigned elemWidth = getStructWidth(*struct_type, 0, false);
                                fieldWidth.emplace_back(elemWidth);
                            } else {
                                errs() << "Each field must be integer or struct type\n";
                                exit(1);
                            }
                        }
                    }
                }
                if (name != "top_intf") {
                    varRegMap[inst] = i;
                    i++;

                    // record the types
                    DIType* varType = varMeta->getType();
                    DICompositeType* varBaseType;
                    std::string varTypeName;
                    if (isa<DIDerivedType>(*varType)) {
                        varTypeName = ((cast<DIDerivedType>(varType))->getName()).str();
                        auto *tmp = (cast<DIDerivedType>(varType))->getBaseType();
                        if (isa<DICompositeType>(*tmp)) {
                            varBaseType = cast<DICompositeType>(tmp);
                        } else {
                            continue;
                        }
                    } else if (isa<DICompositeType>(*varType)) {
                        varBaseType = cast<DICompositeType>(varType);
                        varTypeName = (varBaseType->getName()).str();
                    } else {
                        continue;
                    }
                    if (varTypeMap.find(varTypeName) == varTypeMap.end()) {
                        if (varBaseType->getTag() == 0x0013) {  // The ID of DW_TAG_structure_type
                            varTypeMap[varTypeName] = new std::vector<std::tuple<std::string, int, std::string>>();
                            DINodeArray fieldArray = varBaseType->getElements();
                            int j = 0;
                            for (auto field_it = fieldArray.begin(); field_it != fieldArray.end(); field_it++) {
                                if (isa<DIDerivedType>(**field_it)) {
                                    DIDerivedType* field = cast<DIDerivedType>(*field_it);
                                    if (field->getTag() == 0x000d) { // The ID of DW_TAG_member
                                        auto fieldName = field->getName();
                                        std::string fieldTypeName;
                                        if (isa<DICompositeType>(*(field->getBaseType()))) {
                                            auto *fieldType = cast<DICompositeType>(field->getBaseType());
                                            if (fieldType->getTag() == 0x0013) {
                                                fieldTypeName = fieldType->getName();
                                            }
                                        }
                                        varTypeMap[varTypeName]->emplace_back(fieldName, fieldWidth[j], fieldTypeName);
                                        j++;
                                        // errs() << "Type: " << varTypeName << ", Field: " << fieldName << ", Size: " << field->getSizeInBits() << "\n";
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::string generateStructHeader(int REGWIDTH) {
        std::string program;
        for (auto varType = varTypeMap.begin(); varType != varTypeMap.end(); varType++) {
            program += "typedef struct {\n";
            auto varTypeName = varType->first;
            int totalSize = 0;
            std::string setFunction;
            for (auto field = varType->second->begin(); field != varType->second->end(); field++) {
                std::string fieldName = std::get<0>(*field);
                int width = std::get<1>(*field);
                std::string fieldTypeName = std::get<2>(*field);
                if (fieldTypeName == "") {
                    program += ("    sc_biguint<" + std::to_string(width) + "> " + fieldName + ";\n");
                    setFunction += ("        " + fieldName + " = bv.range(" + std::to_string(totalSize+width-1) +
                        ", " + std::to_string(totalSize) + ");\n");
                    totalSize += width;
                } else {
                    program += ("    " + fieldTypeName + " " + fieldName + ";\n");
                    setFunction += ("        " + fieldName + ".set(bv.range(" + std::to_string(totalSize+width-1) +
                        ", " + std::to_string(totalSize) + "));\n");
                    totalSize += width;
                }
            }
            program += ("    void set(sc_biguint<" + std::to_string(totalSize) + "> bv) {\n");
            program += setFunction;
            program +=  "    }\n"
                        "    sc_biguint<REG_WIDTH> to_uint() {\n"
                        "        sc_biguint<REG_WIDTH> val = (";
            if (totalSize < REGWIDTH) {
                program += "0, ";
            }
            std::string tmp;
            for (auto field = varType->second->rbegin(); field != varType->second->rend(); field++) {
                std::string fieldName = std::get<0>(*field);
                std::string fieldTypeName = std::get<2>(*field);
                if (fieldTypeName == "") {
                    tmp += (fieldName + ", ");
                } else {
                    tmp += (fieldName + ".to_uint(), ");
                }
            }
            program += tmp.substr(0, tmp.length()-2);
            program += ");\n        return val;\n    }\n} ";
            program += (varTypeName + ";\n\n");
        }
        
        return program;
    }

    void initializeBBLoc(Function &F) {
        for (auto bb_it = F.begin(); bb_it != F.end(); bb_it++) {
            for (auto inst_it = bb_it->begin(); inst_it != bb_it->end(); inst_it++) {
                Instruction *inst = dyn_cast<Instruction>(&*inst_it);
                if (DILocation *Loc = inst->getDebugLoc()) {
                    unsigned line = Loc->getLine();
                    unsigned col = Loc->getColumn();
                    if (lineToBBs.find(line) != lineToBBs.end()) {
                        bool found = false;
                        for (auto it = lineToBBs[line]->begin(); it != lineToBBs[line]->end(); it++) {
                            if (it->second == &*bb_it) {
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            (*lineToBBs[line])[col] = &*bb_it;
                        }
                    } else {
                        lineToBBs[line] = new std::map<int, Value*>();
                        (*lineToBBs[line])[col] = &*bb_it;
                    }
                }
            }
        }
    }

    BasicBlock* getBBfromLoc(int lineStart, int colStart, int lineEnd, int colEnd) {
        if (lineToBBs.find(lineStart) != lineToBBs.end()) {
            BasicBlock* lastBB;
            for (auto it = lineToBBs[lineStart]->begin(); it != lineToBBs[lineStart]->end(); it++) {
                if (it->first >= colStart) {
                    return dyn_cast<BasicBlock>(it->second);
                }
            }
        }
        for (int line = lineStart+1; line < lineEnd; line++) {
            if (lineToBBs.find(line) != lineToBBs.end()) {
                for (auto it = lineToBBs[line]->begin(); it != lineToBBs[line]->end(); it++) {
                    return dyn_cast<BasicBlock>(it->second);
                }
            }
        }
        if (lineToBBs.find(lineEnd) != lineToBBs.end()) {
            for (auto it = lineToBBs[lineEnd]->begin(); it != lineToBBs[lineEnd]->end(); it++) {
                if (it->first < colEnd) {
                    return dyn_cast<BasicBlock>(it->second);
                }
            }
        }
        return NULL;
    }

    int initializeProfData() {
        std::ifstream file("prof.json");
        std::string json((std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());

        rapidjson::Document doc;
        doc.Parse(json.c_str());
     
        if (doc.HasParseError()) {
            std::cerr << "Error parsing JSON: "
                 << doc.GetParseError() << std::endl;
            return 1;
        }

        // parse branch regions
        const rapidjson::Value &branches = doc["data"][0]["files"][0]["branches"];
        numProfBranches = branches.Size();
        profBranches = new int*[numProfBranches];
        for (int i = 0; i < numProfBranches; i++) {
            profBranches[i] = new int[9];
            for (int j = 0; j < 9; j++) {
                profBranches[i][j] = branches[i][j].GetInt();
            }
        }

        // parse segments
        filename = doc["data"][0]["files"][0]["filename"].GetString();
        const rapidjson::Value &segments = doc["data"][0]["files"][0]["segments"];

        numProfSegments = segments.Size();
        profSegments = new int*[numProfSegments];
        for (int i = 0; i < numProfSegments; i++) {
            profSegments[i] = new int[6];
            for (int j = 0; j < 3; j++) {
                profSegments[i][j] = segments[i][j].GetInt();
            }
            for (int j = 3; j < 6; j++) {
                profSegments[i][j] = segments[i][j].GetBool();
            }
        }
        numLine = segments[numProfSegments-1][0].GetInt();

        // parse regions
        const rapidjson::Value &functions = doc["data"][0]["functions"];
        for (rapidjson::Value::ConstValueIterator function_it = functions.Begin(); function_it != functions.End(); function_it++) {
            std::string func_name = (*function_it)["name"].GetString();
            if (func_name.find("primate_main") != std::string::npos) {
                const rapidjson::Value &regions = (*function_it)["regions"];
                numProfRegions = regions.Size();
                profRegions = new int*[numProfRegions];
                for (int i = 0; i < numProfRegions; i++) {
                    profRegions[i] = new int[8];
                    for (int j = 0; j < 8; j++) {
                        profRegions[i][j] = regions[i][j].GetInt();
                    }
                }
                break;
            }
        }

        return 0;
    }

    int getSegExeCount(const unsigned line, const unsigned col, int &segId, int &segLineStart, int &segColStart, int &segLineEnd, int &segColEnd) {
        int segExeCount = 0;
        for (int i = 0; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) {
                // skip this entry if it has no count
                continue;
            }
            segLineStart = profSegments[i][0];
            segColStart = profSegments[i][1];
            if ((line < segLineStart) || (line == segLineStart && col < segColStart)) {
                continue;
            } else if (i+1 < numProfSegments) {
                if (line > profSegments[i+1][0] || (line == profSegments[i+1][0] && col >= profSegments[i+1][1])) {
                    continue;
                }
            }

            segExeCount = profSegments[i][2];
            segId = i;
            for (int j = i+1; j < numProfSegments; j++) {
                if (profSegments[j][3] == 0) {
                    // skip this entry if it has no count
                    continue;
                }
                segLineEnd = profSegments[j][0];
                segColEnd = profSegments[j][1];
                break;
            }
            break;
        }
        return segExeCount;
    }

    Function* initializeExeCount(Module &M) {
        numInstPerSeg = new int[numProfSegments];
        for (int i = 0; i < numProfSegments; i++) {
            numInstPerSeg[i] = 0;
        }
        for (Module::iterator mi = M.begin(); mi != M.end(); mi++) {
            if ((mi->getName()).find("primate_main") != StringRef::npos) {
                numTotalInst = 0;
                numTotalExeCount = 0;
                int profSegExeCount;
                int profSegLineStart = 0;
                int profSegColStart = 0;
                int profSegLineEnd = 0;
                int profSegColEnd = 0;
                int segId = 0;
                for (Function::iterator bi = mi->begin(); bi != mi->end(); bi++) {
                    BasicBlock *bb = &*bi;
                    bool newBB = true;
                    int numInstWODbg = 0;
                    for (auto ii = bb->begin(); ii != bb->end(); ++ii) {
                        Instruction *inst = dyn_cast<Instruction>(&*ii);
                        if (!(isa<CallInst>(*inst) || isa<AllocaInst>(*inst) || isa<PHINode>(*inst))) {
                            if (DILocation *Loc = inst->getDebugLoc()) {
                                unsigned line = Loc->getLine();
                                unsigned col = Loc->getColumn();
                                numTotalInst++;
                                if ((line < profSegLineStart) || (line == profSegLineStart && col < profSegColStart) || (line == profSegLineEnd && col >= profSegColEnd) || (line > profSegLineEnd)) {
                                    // Update the count if the instruction is in a new segment
                                    profSegExeCount = getSegExeCount(line, col, segId, profSegLineStart, profSegColStart, profSegLineEnd, profSegColEnd);
                                }
                                numInstPerSeg[segId] += (numInstWODbg + 1);
                                numTotalExeCount += (profSegExeCount * (numInstWODbg + 1));
                                numInstWODbg = 0;
                                newBB = false;
                            } else if (newBB) {
                                numTotalInst++;
                                numInstWODbg++;
                            } else {
                                numTotalInst += 1;
                                numInstPerSeg[segId] += 1;
                                numTotalExeCount += profSegExeCount;
                            }
                        }
                    }
                }
                avgExeCountPerInst = numTotalExeCount / numTotalInst;
                errs() << "Total static count: " << numTotalInst << ", total dynamic count: " << numTotalExeCount << "\n";
                return (&*mi);
            }
        }
        return NULL;
    }

    bool checkInRegion(int line, int col, int lineStart, int colStart, int lineEnd, int colEnd) {
        if (line < lineStart) {
            return false;
        } else if (line == lineStart && col < colStart) {
            return false;
        } else if (line == lineEnd && col > colEnd) {
            return false;
        } else if (line > lineEnd) {
            return false;
        } else {
            return true;
        }
    }

    bool checkRegionEntry(int segId) {
        if (profSegments[segId][4] != 0) {
            return true;
        } else if (segId > 0 && (profSegments[segId-1][0] == profSegments[segId][0]) 
            && (profSegments[segId-1][1] == profSegments[segId][1]) && (profSegments[segId-1][4] != 0)) {
            return true;
        } else {
            return false;
        }
    }

    bool checkRegionExit(int segId) {
        // TODO: searching algorithm can be optimized
        int lineStart = profSegments[segId][0];
        int colStart = profSegments[segId][1];
        for (int i = 0; i < numProfRegions; i++) {
            // skip gap region since they are always kept
            if ((profRegions[i][7] != 3) && (lineStart == profRegions[i][2]) && (colStart == profRegions[i][3])) {
                return true;
            } else if (profRegions[i][0] > lineStart) {
                return false;
            }
        }
        return false;
    }

    bool getRegionCloseLoc(int line, int col, int &lineClose, int &colClose) {
        // TODO: searching algorithm can be optimized
        for (int i = 0; i < numProfRegions; i++) {
            // skip gap region since they are always kept
            if ((profRegions[i][7] != 3) && (line == profRegions[i][0]) && (col == profRegions[i][1])) {
                lineClose = profRegions[i][2];
                colClose = profRegions[i][3];
                return true;
            } else if (profRegions[i][0] > line) {
                return false;
            }
        }
        return false;
    }

    bool checkBranchRegion(int segId) {
        // TODO: searching algorithm can be optimized
        int lineStart = profSegments[segId][0];
        int colStart = profSegments[segId][1];
        for (int i = 0; i < numProfBranches; i++) {
            if (lineStart >= profBranches[i][0] && lineStart <= profBranches[i][2]) {
                if (lineStart == profBranches[i][0] && colStart < profBranches[i][1]) {
                    continue;
                } else if (lineStart == profBranches[i][2] && colStart > profBranches[i][3]) {
                    continue;
                } else {
                    return true;
                }
            } else if (lineStart < profBranches[i][0]) {
                return false;
            }
        }
        return false;
    }

    bool checkKeep(int segId, std::string sourceBuf) {
        if (profSegments[segId][5] != 0) {
            // gap region
            return true;
        } else if (sourceBuf.find("if ") == 0) {
            // starting with "if"
            return true;
        } else if (checkBranchRegion(segId)) {
            // within a branch region or following a branch region
            return true;
        } else {
            return false;
        }
    }

    bool getSource(std::ifstream &inFile, std::string &sourceBuf, int &line, int &col, const int lineEnd, const int colEnd, std::string pattern) {
        // Return true if the pattern is found
        char c;

        while ((line < lineEnd) || (line == lineEnd && col < colEnd)) {
            inFile.get(c);
            if (c == '\n') {
                line++;
                col = 1;
            } else {
                col++;
            }
            sourceBuf += c;
        }

        if (sourceBuf.find(pattern) != std::string::npos) {
            return true;
        }

        return false;
    }

    template<typename generator>
    double nf(double x, generator& g) {
        std::normal_distribution<double> d(0, 0.4);
        double step = d(g);
        // errs() << "step: " << step << "\n";
        double res = x + step;
        if (res < 0.0) {
            res = 0.001;
        }
        return res;
    }

    double exploreFunction(std::ifstream &profile, const double init_th, const int pos, const int segId) {
        double th_old = init_th;
        int count = 200;

        // Simulated annealing algorithm
        double cost_old = evalFunction(profile, th_old, pos, segId);
        double th_best = th_old;
        double cost_best = cost_old;

        std::random_device rd;
        std::mt19937_64 g(rd());

        std::uniform_real_distribution<double> rf(0, 1);

        for (; count > 0; --count) {
            double th_new = nf<decltype(g)>(th_old, g);
            double cost_new = evalFunction(profile, th_new, pos, segId);

            // errs() << "new th: " << th_new << ", new cost: " << cost_new << ". ";

            if (cost_new < cost_best) {
                th_best = th_new;
                cost_best = cost_new;
            }

            if (cost_new < cost_old || std::exp((cost_old - cost_new) / count) > rf(g)) {
                // errs() << "move\n";
                th_old = th_new;
                cost_old = cost_new;
            }
        }
        // errs() << "\n";

        return th_best;
    }

protected:

    virtual double evalFunction(std::ifstream &profile, double th, const int pos, const int segId) = 0;

}; // class PrimateBFUGenBase
}

#endif

