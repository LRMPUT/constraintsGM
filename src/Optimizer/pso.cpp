/// PSO implementation
/// Authors:
/// Dominik Belter

#include "Optimizer/pso.h"
#include <chrono>

using namespace Eigen;

PSO::Ptr psoOptimizer;

PSO::PSO(void) : Optimizer("PSO", TYPE_PSO), mt(std::chrono::high_resolution_clock::now().time_since_epoch().count()) {
    bestFitness = std::numeric_limits<double>::max();
}

PSO::PSO(std::string configFilename) : Optimizer("PSO", TYPE_PSO), mt(std::chrono::high_resolution_clock::now().time_since_epoch().count()) {
    bestFitness = std::numeric_limits<double>::max();
    setlocale(LC_NUMERIC,"C");
    tinyxml2::XMLDocument doc;
    std::string filename = "../../resources/" + configFilename;
    doc.LoadFile( filename.c_str() );
    if (doc.ErrorID())
        std::cout << "unable to load optimizer config file: " << filename << ".\n";
    tinyxml2::XMLElement * model = doc.FirstChildElement( "PSO" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose); //verbose
    model->FirstChildElement( "parameters" )->QueryIntAttribute("N", &N); //number of objective variables/problem dimension
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("c1", &c1); //c1
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("c2", &c2); //c2
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("stopfitness", &stopfitness); //stop if fitness < stopfitness (minimization)
    model->FirstChildElement( "parameters" )->QueryIntAttribute("particlesNo", &particlesNo); //particles no
    model->FirstChildElement( "parameters" )->QueryIntAttribute("maxEpoch", &maxEpoch); //max epoch
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("speedLimit", &speedLimit); //speed limit
    model->FirstChildElement( "parameters" )->QueryIntAttribute("breakIfNoProgressFor", &breakIfNoProgressFor); //stop if no progress for more than specified number of iterations
    range.resize(N);
    distInit.resize(N);
    std::srand((unsigned int)time(NULL));
    for (int dimNo=0;dimNo<N;dimNo++){
        std::string dimName = "dim" + std::to_string(dimNo);
        model->FirstChildElement( dimName.c_str() )->QueryDoubleAttribute("min", &range[dimNo].first);
        model->FirstChildElement( dimName.c_str() )->QueryDoubleAttribute("max", &range[dimNo].second);
        distInit[dimNo] = std::uniform_real_distribution<double>(range[dimNo].first, range[dimNo].second);
    }
    distUnit = std::uniform_real_distribution<double>(0.0,1.0);
}

const std::string& PSO::getName() const {
    return name;
}

/// set initial parameters
void PSO::setParameters() {
    bestFitness = std::numeric_limits<double>::max();
    bestParticle = 0;
    particles.resize(particlesNo);
    size_t particleNo=0;
    for (auto& particle : particles){
        if (injectSolutionFlag&&particleNo==0){
            particle.solution = injectedSolution;
            particle.bestSolution = injectedSolution;
            particle.bestFitness = std::numeric_limits<double>::max();
            particle.fitness = std::numeric_limits<double>::max();
            particle.speed = VectorXd::Zero(N);
            injectSolutionFlag=false;
        }
        else{
            particle.solution = VectorXd::Ones(N);
            particle.bestSolution = VectorXd::Ones(N);
            particle.bestFitness = std::numeric_limits<double>::max();
            particle.fitness = std::numeric_limits<double>::max();
            particle.speed = VectorXd::Zero(N);
            for (size_t dimNo=0;dimNo<(size_t)N;dimNo++){
                particle.solution(dimNo) = distInit[dimNo](mt);
                particle.bestSolution(dimNo) = particle.solution(dimNo);
            }
        }
        particleNo++;
    }
}

/// set boundaries
void PSO::setBoundaries(const std::vector<std::pair<double,double>>& _range) {
    range = _range;
}

/// move particles
void PSO::moveParticles(void){
    size_t particleNo = 0;
    for (auto& particle : particles){
        VectorXd _rand1 = VectorXd::Ones(N);
        VectorXd _rand2 = VectorXd::Ones(N);
        for (size_t dimNo=0;dimNo<(size_t)N;dimNo++){
            _rand1(dimNo) = distUnit(mt);
            _rand2(dimNo) = distUnit(mt);
        }
        VectorXd speed = particle.speed + c1*_rand1*(particle.bestSolution-particle.solution) + c2*_rand2*(particles[bestParticle].bestSolution-particle.solution);
//        std::cout << "particle.solution " << particle.solution.transpose() <<"\n";
//        std::cout << "particle.bestSolution " << particle.bestSolution.transpose() << "\n";
//        std::cout << "particle.speed " << particle.speed.transpose() <<"\n";
//        std::cout << "best particel " << bestParticle << "\n";
//        std::cout << "particles[bestParticle].bestSolution " << particles[bestParticle].bestSolution.transpose() << "\n";
//        std::cout << "c1, c2 " << c1 << ", " << c2 << " _rand " << _rand1.transpose() << " ,,, " << _rand2.transpose() << "\n";
//        std::cout << "particle.solution "<< particle.solution.transpose() << "\n";
//        std::cout << "speed " << speed.transpose() <<"\n";
//        getchar();
        //limit speed
        for (size_t dimNo=0;dimNo<(size_t)N;dimNo++){
            if (fabs(speed(dimNo))>speedLimit){
                if (speed(dimNo)>0)
                    speed(dimNo) = speedLimit;
                else
                    speed(dimNo) = -speedLimit;
            }
        }
        particle.speed = speed;
        particle.solution = particle.solution + speed;
        //check dimensions
        for (size_t dimNo=0;dimNo<(size_t)N;dimNo++){
            if (particle.solution(dimNo)<range[dimNo].first)
                particle.solution(dimNo)=range[dimNo].first;
            else if (particle.solution(dimNo)>range[dimNo].second)
                particle.solution(dimNo)=range[dimNo].second;
        }
        particleNo++;
    }
}

/// optimize
void PSO::Optimize(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function){
    setParameters();
    //Generation Loop
    for (size_t epochNo = 0; epochNo<(size_t)maxEpoch;epochNo++){
        if (verbose)
            std::cout << "Iteration no: " << epochNo << "\n";
        //evaluate each particle
        size_t particleNo = 0;
        bool isProgress(false);
        for (auto& particle : particles){
            particle.fitness = funcMap[function](particle.solution);
            if (particle.fitness<bestFitness){
                bestFitness = particle.fitness;
                bestParticle = particleNo;
            }
            if (particle.fitness<particle.bestFitness){
                particle.bestFitness = particle.fitness;
                particle.bestSolution = particle.solution;
                isProgress=true;
            }
            particleNo++;
        }
        if (!isProgress)
            noProgressIter++;

        //move particles
        moveParticles();

        if (verbose) {
            std::cout << "best parameters:\n";
            std::cout << particles[bestParticle].bestSolution.transpose() << "\n";
            std::cout << "best fitness: " << bestFitness << "\n";
        }

        //Break, if fitness is good enough or condition exceeds 1e14, better termination methods are advisable
        if (bestFitness<stopfitness||noProgressIter>(size_t)breakIfNoProgressFor){
            noProgressIter=0;
            break;
        }
    }
}

/// print output
void PSO::output() {
    //Return best point of last iteration.
    VectorXd xmin(N);
    for (int i = 0; i < N; ++i)
        xmin(i) = particles[bestParticle].bestSolution(i);
    std::cout << "min: " << xmin.transpose() << "\n";
}

/// get result
void PSO::getResult(Eigen::VectorXd& xmin){
    xmin.resize(N);
    for (int i = 0; i < N; ++i)
        xmin(i) = particles[bestParticle].bestSolution(i);
}

std::unique_ptr<Optimizer> createPSO(void) {
    return walkers::make_unique<PSO>();
}

std::unique_ptr<Optimizer> createPSO(std::string configFile){
    return walkers::make_unique<PSO>(configFile);
}
