//  PrimateBFUGen.cpp
//  To generate Blue Function Units for Primate
//
//  Author: Rui Ma
/////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************

Liveness: OUT[n] = UNION_{s E succ[n]} IN[s]  //meet
IN[n] = GEN[n] U (OUT[n] - KILL[n]) //transfer function

Flow Direction: Backward
A BitVector stored at each node for IN and OUT. Bit vector contains an entry for all the values

Boundary Conditions: empty set for flow value. identified by no successors.

 *********************************************************************************/

#include<llvm/Pass.h>
#include<llvm/IR/Function.h>
#include<llvm/IR/Module.h>
#include<llvm/IR/Metadata.h>
#include<llvm/Support/raw_ostream.h>
#include<llvm/Support/FormattedStream.h>
#include<llvm/IR/InstIterator.h>
#include<llvm/IR/Instruction.h>
#include<llvm/IR/AssemblyAnnotationWriter.h>
#include<llvm/ADT/BitVector.h>
#include<llvm/IR/ValueMap.h>
#include<llvm/ADT/DenseMap.h>

#include<llvm/IR/LegacyPassManager.h>
#include<llvm/Transforms/IPO/PassManagerBuilder.h>

#include "inputGen.h"
#include "outputGen.h"
#include "inputSimpleGen.h"

using namespace llvm;

namespace {

    class PrimateInputGen : public ModulePass, public inputGen {

        public:
            static char ID;

            // set forward false in the constructor DataFlow()
            PrimateInputGen() : PrimateBase(), ModulePass(ID) {}

            void initializeVarNames(Function &F) {
                initializePrimateVars(F);
                initializeVarNameMap(F);
            }
            
            virtual bool runOnModule(Module &M){
                initializeProfData();
                Function *main_func = initializeExeCount(M);
                initializeBBLoc(*main_func);
                initializeVarNames(*main_func);

                BasicBlock *startBB, *endBB;
                inputGen::inferInputFunctions(*main_func, startBB, endBB);
                extractInputFunction(*main_func, endBB);

                return false;
            }

            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesAll();
            }

    };

    class PrimateOutputGen : public ModulePass, public outputGen {

        public:
            static char ID;

            // set forward false in the constructor DataFlow()
            PrimateOutputGen() : PrimateBase(), ModulePass(ID) {}

            void initializeVarNames(Function &F) {
                initializePrimateVars(F);
                initializeVarNameMap(F);
            }
            
            virtual bool runOnModule(Module &M){
                initializeProfData();
                Function *main_func = initializeExeCount(M);
                initializeBBLoc(*main_func);
                initializeVarNames(*main_func);

                BasicBlock *startBB, *endBB;
                outputGen::inferOutputFunctions(*main_func, startBB, endBB);
                outputGen::extractOutputFunction(*main_func, endBB);

                return false;
            }

            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesAll();
            }

    };

    class PrimateInputSimpleGen : public ModulePass, public inputSimpleGen {

        public:
            static char ID;

            // set forward false in the constructor DataFlow()
            PrimateInputSimpleGen() : PrimateBase(), ModulePass(ID) {}

            void initializeVarNames(Function &F) {
                initializePrimateVars(F);
                initializeVarNameMap(F);
            }
            
            virtual bool runOnModule(Module &M){
                initializeProfData();
                Function *main_func = initializeExeCount(M);
                initializeBBLoc(*main_func);
                initializeVarNames(*main_func);

                inputSimpleGen::extractInputFunction(*main_func);

                return false;
            }

            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesAll();
            }

    };

    char PrimateInputGen::ID = 0;
    char PrimateOutputGen::ID = 1;
    char PrimateInputSimpleGen::ID = 2;

    static RegisterPass<PrimateInputGen> X("inputGen", "primate pass");
    static RegisterPass<PrimateOutputGen> Y("outputGen", "primate pass");
    static RegisterPass<PrimateInputSimpleGen> Z("inputSimpleGen", "primate pass");

    // static RegisterStandardPasses Y(
    // PassManagerBuilder::EP_EarlyAsPossible,
    // [](const PassManagerBuilder &Builder,
    //    legacy::PassManagerBase &PM) { PM.add(new PrimateBFUGen()); });

}

