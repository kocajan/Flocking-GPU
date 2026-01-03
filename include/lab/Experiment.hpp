

struct ExperimentContext {
    const Config& experimentConfig;
    SimState& simState;
    Config& simConfig;
    int totalTicks;
    int warmupTicks;
};

class Experiment {
public:
    virtual ~Experiment() = default;

    // Called before any runs begin
    virtual void onExperimentStart(const ExperimentContext&) {}

    // Called when switching to a new version
    virtual void onVersionStart(const std::string& version,
                                const ExperimentContext&) {}

    // Called before running a boid-count batch
    virtual void onBoidConfigStart(int boidCount,
                                   const ExperimentContext&) {}

    // Called after each simulated step (may be ignored)
    virtual void onTick(int tick,
                        double stepMs,
                        const ExperimentContext&) {}

    // Called after finishing a boid-count batch
    virtual void onBoidConfigEnd(int boidCount,
                                 const ExperimentContext&) {}

    // Called after finishing a version
    virtual void onVersionEnd(const std::string& version,
                              const ExperimentContext&) {}

    // Called at the very end
    virtual void onExperimentEnd(const ExperimentContext&) {}

    // Provide access to collected results in any structure
    virtual void finalizeResults(const Config& experimentConfig) = 0;
};
