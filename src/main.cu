#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>

#include "flamegpu/flame_api.h"
#include "flamegpu/runtime/flamegpu_api.h"
#include "flamegpu/io/factory.h"
#include "flamegpu/util/nvtx.h"

#define PRED_PREY_INTERACTION_RADIUS 0.1f
#define SAME_SPECIES_AVOIDANCE_RADIUS 0.035f
#define DELTA_TIME 0.001f
#define PRED_SPEED_ADVANTAGE 2.0f
#define PRED_KILL_DISTANCE 0.02f
#define BOUNDS_WIDTH 2.0f
#define MIN_POSITION -1.0f
#define MAX_POSITION 1.0f
#define PREY_GROUP_COHESION_RADIUS 0.2f
#define GRASS_EAT_DISTANCE 0.02f
#define GRASS_REGROW_CYCLES 100
#define GAIN_FROM_FOOD_PREY 75

typedef struct CSVRow {
    int preyPop;
    int predatorPop;
    int grassPop;
} CSVRow;

std::vector<CSVRow> csvData;

FLAMEGPU_STEP_FUNCTION(recordPopulation) {
    CSVRow row;
    row.predatorPop = FLAMEGPU->agent("predator").count();
    row.preyPop = FLAMEGPU->agent("prey").count();
    row.grassPop = FLAMEGPU->agent("grass").count<int>("available", 1);
    csvData.push_back(row);
}

FLAMEGPU_EXIT_FUNCTION(savePopulationData) {
    std::ofstream outputFile;
    outputFile.open("iterations/PreyPred_Count.csv");
    if (outputFile.is_open()) {
        for (const CSVRow& csvRow : csvData) {
            outputFile << "Prey, " << csvRow.preyPop << ", Predator," << csvRow.predatorPop << ", Grass," << csvRow.grassPop << std::endl;
        }
    }
    else {
        std::cout << "Failed to open file for saving population data!";
    }
    std::cout << "Data saved" << std::endl;
}

CSVRow loadPopulations() {
    std::ifstream inputFile("iterations/initial_populations.txt");
    CSVRow initialPopulations;
    initialPopulations.preyPop = 800;
    initialPopulations.predatorPop = 400;
    initialPopulations.grassPop = 0;
    if (inputFile.is_open()) {
        inputFile >> initialPopulations.preyPop >> initialPopulations.predatorPop >> initialPopulations.grassPop;
    }
    else {
        std::cout << "Warning: Failed to open initial_populations.txt, using default population values" << std::endl;
    }
    return initialPopulations;
}

/*
   The following section of code defines the agent function behaviours in the following format:

   FLAMEGPU_AGENT_FUNCTION(function_name, input_message_type, output_message_type) {
       behaviour definition goes here
   }

*/

// Predator functions
FLAMEGPU_AGENT_FUNCTION(pred_output_location, MsgNone, MsgBruteForce) {
    const float id = FLAMEGPU->getVariable<int>("id");
    const float x = FLAMEGPU->getVariable<float>("x");
    const float y = FLAMEGPU->getVariable<float>("y");
    FLAMEGPU->message_out.setVariable<int>("id", id);
    FLAMEGPU->message_out.setVariable<float>("x", x);
    FLAMEGPU->message_out.setVariable<float>("y", y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_follow_prey, MsgBruteForce, MsgNone) {
    // Fetch the predator's position
    const float predator_x = FLAMEGPU->getVariable<float>("x");
    const float predator_y = FLAMEGPU->getVariable<float>("y");

    // Find the closest prey by iterating the prey_location messages
    float closest_prey_x = 0.0f;
    float closest_prey_y = 0.0f;
    float closest_prey_distance = PRED_PREY_INTERACTION_RADIUS;
    int is_a_prey_in_range = 0;

    for (const auto& msg : FLAMEGPU->message_in) {
        // Fetch prey location
        const float prey_x = msg.getVariable<float>("x");
        const float prey_y = msg.getVariable<float>("y");

        // Check if prey is within sight range of predator
        const float dx = predator_x - prey_x;
        const float dy = predator_y - prey_y;
        const float separation = sqrt(dx * dx + dy * dy);

        if (separation < closest_prey_distance) {
            closest_prey_x = prey_x;
            closest_prey_y = prey_y;
            closest_prey_distance = separation;
            is_a_prey_in_range = 1;
        }
    }

    // If there was a prey in range, steer the predator towards it
    if (is_a_prey_in_range) {
        const float steer_x = closest_prey_x - predator_x;
        const float steer_y = closest_prey_y - predator_y;
        FLAMEGPU->setVariable<float>("steer_x", steer_x);
        FLAMEGPU->setVariable<float>("steer_y", steer_y);
    }

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_avoid, MsgBruteForce, MsgNone) {
    // Fetch this predator's position
    const float predator_x = FLAMEGPU->getVariable<float>("x");
    const float predator_y = FLAMEGPU->getVariable<float>("y");
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;

    // Add a steering factor away from each other predator. Strength increases with closeness.
    for (const auto& msg : FLAMEGPU->message_in) {
        // Fetch location of other predator
        const float other_predator_x = msg.getVariable<float>("x");
        const float other_predator_y = msg.getVariable<float>("y");

        // Check if the two predators are within interaction radius
        const float dx = predator_x - other_predator_x;
        const float dy = predator_y - other_predator_y;
        const float separation = sqrt(dx * dx + dy * dy);

        if (separation < SAME_SPECIES_AVOIDANCE_RADIUS && separation > 0.0f) {
            avoid_velocity_x += SAME_SPECIES_AVOIDANCE_RADIUS / separation * dx;
            avoid_velocity_y += SAME_SPECIES_AVOIDANCE_RADIUS / separation * dy;
        }
    }

    float steer_x = FLAMEGPU->getVariable<float>("steer_x");
    float steer_y = FLAMEGPU->getVariable<float>("steer_y");
    steer_x += avoid_velocity_x;
    steer_y += avoid_velocity_y;
    FLAMEGPU->setVariable<float>("steer_x", steer_x);
    FLAMEGPU->setVariable<float>("steer_y", steer_y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_move, MsgNone, MsgNone) {
    float predator_x = FLAMEGPU->getVariable<float>("x");
    float predator_y = FLAMEGPU->getVariable<float>("y");
    float predator_vx = FLAMEGPU->getVariable<float>("vx");
    float predator_vy = FLAMEGPU->getVariable<float>("vy");
    const float predator_steer_x = FLAMEGPU->getVariable<float>("steer_x");
    const float predator_steer_y = FLAMEGPU->getVariable<float>("steer_y");
    const float predator_life = FLAMEGPU->getVariable<int>("life");

    // Integrate steering forces and cap velocity
    predator_vx += predator_steer_x;
    predator_vy += predator_steer_y;

    float speed = sqrt(predator_vx * predator_vx + predator_vy * predator_vy);
    if (speed > 1.0f) {
        predator_vx /= speed;
        predator_vy /= speed;
    }

    // Integrate velocity
    predator_x += predator_vx * DELTA_TIME * PRED_SPEED_ADVANTAGE;
    predator_y += predator_vy * DELTA_TIME * PRED_SPEED_ADVANTAGE;

    // Bound the position within the environment 
    predator_x = predator_x < MIN_POSITION ? MIN_POSITION : predator_x;
    predator_x = predator_x > MAX_POSITION ? MAX_POSITION : predator_x;
    predator_y = predator_y < MIN_POSITION ? MIN_POSITION : predator_y;
    predator_y = predator_y > MAX_POSITION ? MAX_POSITION : predator_y;

    // Update agent state
    FLAMEGPU->setVariable<float>("x", predator_x);
    FLAMEGPU->setVariable<float>("y", predator_y);
    FLAMEGPU->setVariable<float>("vx", predator_vx);
    FLAMEGPU->setVariable<float>("vy", predator_vy);

    // Reduce life by one unit of energy
    FLAMEGPU->setVariable<int>("life", predator_life - 1);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_eat_or_starve, MsgBruteForce, MsgNone) {
    const int predator_id = FLAMEGPU->getVariable<int>("id");
    int predator_life = FLAMEGPU->getVariable<int>("life");
    int isDead = 0;

    // Iterate prey_eaten messages to see if this predator ate a prey
    for (const auto& msg : FLAMEGPU->message_in) {
        if (msg.getVariable<int>("pred_id") == predator_id) {
            predator_life += FLAMEGPU->environment.get<int>("GAIN_FROM_FOOD_PREDATOR");
        }
    }

    // Update agent state
    FLAMEGPU->setVariable<int>("life", predator_life);

    // Did the predator starve?
    if (predator_life < 1) {
        isDead = 1;
    }

    return isDead ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(pred_reproduction, MsgNone, MsgNone) {
    float random = FLAMEGPU->random.uniform<float>();
    const int currentLife = FLAMEGPU->getVariable<int>("life");
    if (random < FLAMEGPU->environment.get<float>("REPRODUCE_PREDATOR_PROB")) {
        int id = FLAMEGPU->random.uniform<float>() * (float)INT_MAX;
        float x = FLAMEGPU->random.uniform<float>() * BOUNDS_WIDTH - BOUNDS_WIDTH / 2.0f;
        float y = FLAMEGPU->random.uniform<float>() * BOUNDS_WIDTH - BOUNDS_WIDTH / 2.0f;
        float vx = FLAMEGPU->random.uniform<float>() * 2 - 1;
        float vy = FLAMEGPU->random.uniform<float>() * 2 - 1;

        FLAMEGPU->setVariable<int>("life", currentLife / 2);

        FLAMEGPU->agent_out.setVariable<int>("id", id);
        FLAMEGPU->agent_out.setVariable<float>("x", x);
        FLAMEGPU->agent_out.setVariable<float>("y", y);
        FLAMEGPU->agent_out.setVariable<float>("type", 0.0f);
        FLAMEGPU->agent_out.setVariable<float>("vx", vx);
        FLAMEGPU->agent_out.setVariable<float>("vy", vy);
        FLAMEGPU->agent_out.setVariable<float>("steer_x", 0.0f);
        FLAMEGPU->agent_out.setVariable<float>("steer_y", 0.0f);
        FLAMEGPU->agent_out.setVariable<int>("life", currentLife / 2);

    }
    return ALIVE;
}

// Prey functions

FLAMEGPU_AGENT_FUNCTION(prey_output_location, MsgNone, MsgBruteForce) {
    const float id = FLAMEGPU->getVariable<int>("id");
    const float x = FLAMEGPU->getVariable<float>("x");
    const float y = FLAMEGPU->getVariable<float>("y");
    FLAMEGPU->message_out.setVariable<int>("id", id);
    FLAMEGPU->message_out.setVariable<float>("x", x);
    FLAMEGPU->message_out.setVariable<float>("y", y);
    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_avoid_pred, MsgBruteForce, MsgNone) {
    // Fetch this prey's position
    const float prey_x = FLAMEGPU->getVariable<float>("x");
    const float prey_y = FLAMEGPU->getVariable<float>("y");
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;

    // Add a steering factor away from each predator. Strength increases with closeness.
    for (const auto& msg : FLAMEGPU->message_in) {
        // Fetch location of predator
        const float predator_x = msg.getVariable<float>("x");
        const float predator_y = msg.getVariable<float>("y");

        // Check if the two predators are within interaction radius
        const float dx = prey_x - predator_x;
        const float dy = prey_y - predator_y;
        const float distance = sqrt(dx * dx + dy * dy);

        if (distance < PRED_PREY_INTERACTION_RADIUS) {
            // Steer the prey away from the predator
            avoid_velocity_x += (PRED_PREY_INTERACTION_RADIUS / distance) * dx;
            avoid_velocity_y += (PRED_PREY_INTERACTION_RADIUS / distance) * dy;
        }
    }

    // Update agent state 
    FLAMEGPU->setVariable<float>("steer_x", avoid_velocity_x);
    FLAMEGPU->setVariable<float>("steer_y", avoid_velocity_y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_flock, MsgBruteForce, MsgNone) {
    const int   prey_id = FLAMEGPU->getVariable<int>("id");
    const float prey_x = FLAMEGPU->getVariable<float>("x");
    const float prey_y = FLAMEGPU->getVariable<float>("y");

    float group_centre_x = 0.0f;
    float group_centre_y = 0.0f;
    float group_velocity_x = 0.0f;
    float group_velocity_y = 0.0f;
    float avoid_velocity_x = 0.0f;
    float avoid_velocity_y = 0.0f;
    int group_centre_count = 0;

    for (const auto& msg : FLAMEGPU->message_in) {
        const int   other_prey_id = msg.getVariable<int>("id");
        const float other_prey_x = msg.getVariable<float>("x");
        const float other_prey_y = msg.getVariable<float>("y");
        const float dx = prey_x - other_prey_x;
        const float dy = prey_y - other_prey_y;
        const float separation = sqrt(dx * dx + dy * dy);

        if (separation < PREY_GROUP_COHESION_RADIUS && prey_id != other_prey_id) {
            group_centre_x += other_prey_x;
            group_centre_y += other_prey_y;
            group_centre_count += 1;

            // Avoidance behaviour
            if (separation < SAME_SPECIES_AVOIDANCE_RADIUS) {
                // Was a check for separation > 0 in original - redundant?
                avoid_velocity_x += SAME_SPECIES_AVOIDANCE_RADIUS / separation * dx;
                avoid_velocity_y += SAME_SPECIES_AVOIDANCE_RADIUS / separation * dy;
            }
        }
    }

    // Compute group centre as the average of the nearby prey positions and a velocity to move towards the group centre
    if (group_centre_count > 0) {
        group_centre_x /= group_centre_count;
        group_centre_y /= group_centre_count;
        group_velocity_x = group_centre_x - prey_x;
        group_velocity_y = group_centre_y - prey_y;
    }

    float prey_steer_x = FLAMEGPU->getVariable<float>("steer_x");
    float prey_steer_y = FLAMEGPU->getVariable<float>("steer_y");
    prey_steer_x += group_velocity_x + avoid_velocity_x;
    prey_steer_y += group_velocity_y + avoid_velocity_y;
    FLAMEGPU->setVariable<float>("steer_x", prey_steer_x);
    FLAMEGPU->setVariable<float>("steer_y", prey_steer_y);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_move, MsgNone, MsgNone) {
    float prey_x = FLAMEGPU->getVariable<float>("x");
    float prey_y = FLAMEGPU->getVariable<float>("y");
    float prey_vx = FLAMEGPU->getVariable<float>("vx");
    float prey_vy = FLAMEGPU->getVariable<float>("vy");
    const float prey_steer_x = FLAMEGPU->getVariable<float>("steer_x");
    const float prey_steer_y = FLAMEGPU->getVariable<float>("steer_y");
    const float prey_life = FLAMEGPU->getVariable<int>("life");

    // Integrate steering forces and cap velocity
    prey_vx += prey_steer_x;
    prey_vy += prey_steer_y;

    float speed = sqrt(prey_vx * prey_vx + prey_vy * prey_vy);
    if (speed > 1.0f) {
        prey_vx /= speed;
        prey_vy /= speed;
    }

    // Integrate velocity
    prey_x += prey_vx * DELTA_TIME;
    prey_y += prey_vy * DELTA_TIME;

    // Bound the position within the environment - can this be moved
    prey_x = prey_x < MIN_POSITION ? MIN_POSITION : prey_x;
    prey_x = prey_x > MAX_POSITION ? MAX_POSITION : prey_x;
    prey_y = prey_y < MIN_POSITION ? MIN_POSITION : prey_y;
    prey_y = prey_y > MAX_POSITION ? MAX_POSITION : prey_y;


    // Update agent state
    FLAMEGPU->setVariable<float>("x", prey_x);
    FLAMEGPU->setVariable<float>("y", prey_y);
    FLAMEGPU->setVariable<float>("vx", prey_vx);
    FLAMEGPU->setVariable<float>("vy", prey_vy);

    // Reduce life by one unit of energy
    FLAMEGPU->setVariable<int>("life", prey_life - 1);

    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_eaten, MsgBruteForce, MsgBruteForce) {
    int eaten = 0;
    int predator_id = -1;
    float closest_pred = PRED_KILL_DISTANCE;
    const float prey_x = FLAMEGPU->getVariable<float>("x");
    const float prey_y = FLAMEGPU->getVariable<float>("y");

    // Iterate predator_location messages to find the closest predator
    for (const auto& msg : FLAMEGPU->message_in) {
        // Fetch location of predator
        const float predator_x = msg.getVariable<float>("x");
        const float predator_y = msg.getVariable<float>("y");

        // Check if the two predators are within interaction radius
        const float dx = prey_x - predator_x;
        const float dy = prey_y - predator_y;
        const float distance = sqrt(dx * dx + dy * dy);

        if (distance < closest_pred) {
            predator_id = msg.getVariable<int>("id");
            closest_pred = distance;
            eaten = 1;
        }
    }

    if (eaten) {
        FLAMEGPU->message_out.setVariable<int>("id", FLAMEGPU->getVariable<int>("id"));
        FLAMEGPU->message_out.setVariable<int>("pred_id", predator_id);
    }

    return eaten ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_eat_or_starve, MsgBruteForce, MsgNone) {
    // Exercise 3.3
    int isDead = 0;
    const int id = FLAMEGPU->getVariable<int>("id");
    const int life = FLAMEGPU->getVariable<int>("life");

    // Iterate the grass eaten messages 
    for (const auto& msg : FLAMEGPU->message_in)
    {
        // If the grass eaten message indicates that this prey ate some grass then increase the preys life by adding energy
        if (id == msg.getVariable<int>("prey_id")) {
            FLAMEGPU->setVariable<int>("life", life + GAIN_FROM_FOOD_PREY);
        }
    }

    // If the life has reduced to 0 then the prey should die or starvation 
    if (FLAMEGPU->getVariable<int>("life") < 1)
        isDead = 1;

    return isDead ? DEAD : ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(prey_reproduction, MsgNone, MsgNone) {
    float random = FLAMEGPU->random.uniform<float>();
    const int currentLife = FLAMEGPU->getVariable<int>("life");
    if (random < FLAMEGPU->environment.get<float>("REPRODUCE_PREY_PROB")) {
        int id = FLAMEGPU->random.uniform<float>() * (float)INT_MAX;
        float x = FLAMEGPU->random.uniform<float>() * BOUNDS_WIDTH - BOUNDS_WIDTH / 2.0f;
        float y = FLAMEGPU->random.uniform<float>() * BOUNDS_WIDTH - BOUNDS_WIDTH / 2.0f;
        float vx = FLAMEGPU->random.uniform<float>() * 2 - 1;
        float vy = FLAMEGPU->random.uniform<float>() * 2 - 1;

        FLAMEGPU->setVariable<int>("life", currentLife / 2);

        FLAMEGPU->agent_out.setVariable<int>("id", id);
        FLAMEGPU->agent_out.setVariable<float>("x", x);
        FLAMEGPU->agent_out.setVariable<float>("y", y);
        FLAMEGPU->agent_out.setVariable<float>("type", 1.0f);
        FLAMEGPU->agent_out.setVariable<float>("vx", vx);
        FLAMEGPU->agent_out.setVariable<float>("vy", vy);
        FLAMEGPU->agent_out.setVariable<float>("steer_x", 0.0f);
        FLAMEGPU->agent_out.setVariable<float>("steer_y", 0.0f);
        FLAMEGPU->agent_out.setVariable<int>("life", currentLife / 2);

    }
    return ALIVE;
}

// Grass functions
FLAMEGPU_AGENT_FUNCTION(grass_output_location, MsgNone, MsgBruteForce) {
    // Exercise 3.1 : Set the variables for the grass_location message
    const int id = FLAMEGPU->getVariable<int>("id");
    const float x = FLAMEGPU->getVariable<float>("x");
    const float y = FLAMEGPU->getVariable<float>("y");
    FLAMEGPU->message_out.setVariable<int>("id", id);
    FLAMEGPU->message_out.setVariable<float>("x", x);
    FLAMEGPU->message_out.setVariable<float>("y", y);
    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(grass_eaten, MsgBruteForce, MsgBruteForce) {
    // Exercise 3.2
    const float grass_x = FLAMEGPU->getVariable<float>("x");
    const float grass_y = FLAMEGPU->getVariable<float>("y");
    int available = FLAMEGPU->getVariable<int>("available");
    if (available) { 

        int prey_id = -1;
        float closest_prey = GRASS_EAT_DISTANCE;
        int eaten = 0;

        // Iterate predator_location messages to find the closest predator
        for (const auto& msg : FLAMEGPU->message_in) {
            // Fetch location of prey
            const float prey_x = msg.getVariable<float>("x");
            const float prey_y = msg.getVariable<float>("y");

            // Check if the two preys are within interaction radius
            const float dx = grass_x - prey_x;
            const float dy = grass_y - prey_y;
            const float distance = sqrt(dx*dx + dy*dy);

            if (distance < closest_prey) {
                prey_id = msg.getVariable<int>("id");
                closest_prey= distance;
                eaten = 1;
            }
        }

        if (eaten) {
            // Add grass eaten message
            FLAMEGPU->message_out.setVariable<int>("id", FLAMEGPU->getVariable<int>("id"));
            FLAMEGPU->message_out.setVariable<int>("prey_id", prey_id);
           
            // Update grass agent variables
            FLAMEGPU->setVariable<int>("dead_cycles", 0);
            FLAMEGPU->setVariable<int>("available", 0);
            FLAMEGPU->setVariable<float>("type", 3.0f);
        }
    }
    return ALIVE;
}

FLAMEGPU_AGENT_FUNCTION(grass_growth, MsgNone, MsgNone) {
    // Exercise 3.4 
    const float dead_cycles = FLAMEGPU->getVariable<int>("dead_cycles");
    int new_dead_cycles = dead_cycles + 1;
    if (dead_cycles == GRASS_REGROW_CYCLES) {
        FLAMEGPU->setVariable<int>("dead_cycles", 0);
        FLAMEGPU->setVariable<int>("available", 1);
        FLAMEGPU->setVariable<float>("type", 2.0f);
    } 

    const int available = FLAMEGPU->getVariable<int>("available");
    if (available == 0) {
        FLAMEGPU->setVariable<int>("dead_cycles", new_dead_cycles);
    } 

    return ALIVE;
}


// Model definition


int main(int argc, const char** argv) {
    NVTX_RANGE("main");
    NVTX_PUSH("ModelDescription");
    ModelDescription model("Tutorial_PredatorPrey_Example");

    /**
     * MESSAGE DEFINITIONS
     */

    {   // Grass location message
        MsgBruteForce::Description& message = model.newMessage("grass_location_message");
        message.newVariable<int>("id");
        message.newVariable<float>("x");
        message.newVariable<float>("y");
    }
    {   // Predator location message
        MsgBruteForce::Description& message = model.newMessage("predator_location_message");
        message.newVariable<int>("id");
        message.newVariable<float>("x");
        message.newVariable<float>("y");
    }
    {   // Prey location message
        MsgBruteForce::Description& message = model.newMessage("prey_location_message");
        message.newVariable<int>("id");
        message.newVariable<float>("x");
        message.newVariable<float>("y");
    }
    {   // Grass eaten message
        MsgBruteForce::Description& message = model.newMessage("grass_eaten_message");
        message.newVariable<int>("id");
        message.newVariable<int>("prey_id");
    }
    {   // Prey eaten message
        MsgBruteForce::Description& message = model.newMessage("prey_eaten_message");
        message.newVariable<int>("id");
        message.newVariable<int>("pred_id");
    }


    /**
     * AGENT DEFINITIONS
     */

    {   // Prey agent
        AgentDescription& agent = model.newAgent("prey");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
        agent.newVariable<float>("y");
        agent.newVariable<float>("vx");
        agent.newVariable<float>("vy");
        agent.newVariable<float>("steer_x");
        agent.newVariable<float>("steer_y");
        agent.newVariable<int>("life");
        agent.newVariable<float>("type")
            ;
        auto& fn = agent.newFunction("prey_output_location", prey_output_location);
        fn.setMessageOutput("prey_location_message");
        agent.newFunction("prey_avoid_pred", prey_avoid_pred).setMessageInput("predator_location_message");
        agent.newFunction("prey_flock", prey_flock).setMessageInput("prey_location_message");
        agent.newFunction("prey_move", prey_move);
        auto& function = agent.newFunction("prey_eaten", prey_eaten);
        function.setMessageInput("predator_location_message");
        function.setMessageOutput("prey_eaten_message");
        function.setMessageOutputOptional(true);
        function.setAllowAgentDeath(true);
        auto& fn_prey_eat_or_starve = agent.newFunction("prey_eat_or_starve", prey_eat_or_starve);
        fn_prey_eat_or_starve.setMessageInput("grass_eaten_message");
        fn_prey_eat_or_starve.setAllowAgentDeath(true);
        auto& fn_prey_reproduction = agent.newFunction("prey_reproduction", prey_reproduction);
        fn_prey_reproduction.setAgentOutput("prey", "default");
    }

    {   // Predator agent
        AgentDescription& agent = model.newAgent("predator");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
        agent.newVariable<float>("y");
        agent.newVariable<float>("vx");
        agent.newVariable<float>("vy");
        agent.newVariable<float>("steer_x");
        agent.newVariable<float>("steer_y");
        agent.newVariable<int>("life");
        agent.newVariable<float>("type");

        agent.newFunction("pred_output_location", pred_output_location).setMessageOutput("predator_location_message");
        agent.newFunction("pred_follow_prey", pred_follow_prey).setMessageInput("prey_location_message");
        agent.newFunction("pred_avoid", pred_avoid).setMessageInput("predator_location_message");
        agent.newFunction("pred_move", pred_move);
        auto& fn_pred_eat_or_starve = agent.newFunction("pred_eat_or_starve", pred_eat_or_starve);
        fn_pred_eat_or_starve.setMessageInput("prey_eaten_message");
        fn_pred_eat_or_starve.setAllowAgentDeath(true);
        auto& fn_pred_reproduction = agent.newFunction("pred_reproduction", pred_reproduction);
        fn_pred_reproduction.setAgentOutput("predator", "default");
    }

    {   // Grass agent
        AgentDescription& agent = model.newAgent("grass");
        agent.newVariable<int>("id");
        agent.newVariable<float>("x");
        agent.newVariable<float>("y");
        agent.newVariable<int>("dead_cycles");
        agent.newVariable<int>("available");
        agent.newVariable<float>("type");
        auto& fn = agent.newFunction("grass_output_location", grass_output_location);
        fn.setMessageOutput("grass_location_message");
        fn.setMessageOutputOptional(true);
        auto& fn_grass_eaten = agent.newFunction("grass_eaten", grass_eaten);
        fn_grass_eaten.setMessageInput("prey_location_message");
        fn_grass_eaten.setMessageOutput("grass_eaten_message");
        fn_grass_eaten.setMessageOutputOptional(true);
        fn_grass_eaten.setAllowAgentDeath(true);
        agent.newFunction("grass_growth", grass_growth);

    }

    /**
      * ENVIRONMENT VARIABLES
      */

    EnvironmentDescription& env = model.Environment();
    env.add<float>("REPRODUCE_PREY_PROB", 0.05f);
    env.add<float>("REPRODUCE_PREDATOR_PROB", 0.03f);
    env.add<int>("GAIN_FROM_FOOD_PREDATOR", 50);

    /**
     * Control flow
     */
    {   // Layer #1
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(prey_output_location);
        layer.addAgentFunction(pred_output_location);
        layer.addAgentFunction(grass_output_location);
    }
    {   // Layer #2
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(pred_follow_prey);
        layer.addAgentFunction(prey_avoid_pred);
    }
    {   // Layer #3
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(prey_flock);
        layer.addAgentFunction(pred_avoid);
    }
    {   // Layer #4
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(prey_move);
        layer.addAgentFunction(pred_move);
    }
    {   // Layer #5
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(grass_eaten);
        layer.addAgentFunction(prey_eaten);
    }
    {   // Layer #6
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(prey_eat_or_starve);
        layer.addAgentFunction(pred_eat_or_starve);
    }
    {   // Layer #7
        LayerDescription& layer = model.newLayer();
        layer.addAgentFunction(pred_reproduction);
        layer.addAgentFunction(prey_reproduction);
        layer.addAgentFunction(grass_growth);
    }

    model.addStepFunction(recordPopulation);
    model.addExitFunction(savePopulationData);
    NVTX_POP();

    /**
     * Create Model Runner
     */
    NVTX_PUSH("CUDAAgentModel creation");
    CUDAAgentModel cuda_model(model);
    NVTX_POP();

    /**
     * Initialisation
     */
    cuda_model.initialise(argc, argv);

    if (cuda_model.getSimulationConfig().xml_input_file.empty()) {
        printf("XML Input was empty!");
    }

    // Initialise random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> floatDist(-1.0f, 1.0f);
    std::uniform_real_distribution<> predLifeDist(0, 40);
    std::uniform_real_distribution<> preyLifeDist(0, 50);

    // Load initial population data
    CSVRow initialPops = loadPopulations();

    // Initialise predator agents
    int numPredators = initialPops.predatorPop;
    AgentPopulation predatorPopulation(model.Agent("predator"), numPredators);
    for (int i = 0; i < numPredators; i++) {
        AgentInstance predator = predatorPopulation.getNextInstance();
        predator.setVariable<int>("id", i);
        predator.setVariable<float>("x", floatDist(gen));
        predator.setVariable<float>("y", floatDist(gen));
        predator.setVariable<float>("vx", floatDist(gen));
        predator.setVariable<float>("vy", floatDist(gen));
        predator.setVariable<float>("steer_x", 0.0f);
        predator.setVariable<float>("steer_y", 0.0f);
        predator.setVariable<float>("type", 0.0f);
        predator.setVariable<int>("life", predLifeDist(gen));
    }

    // Initialise prey agents 
    int numPrey = initialPops.preyPop;
    AgentPopulation preyPopulation(model.Agent("prey"), numPrey);
    for (int i = 0; i < numPrey; i++) {
        AgentInstance prey = preyPopulation.getNextInstance();
        prey.setVariable<int>("id", i);
        prey.setVariable<float>("x", floatDist(gen));
        prey.setVariable<float>("y", floatDist(gen));
        prey.setVariable<float>("vx", floatDist(gen));
        prey.setVariable<float>("vy", floatDist(gen));
        prey.setVariable<float>("steer_x", 0.0f);
        prey.setVariable<float>("steer_y", 0.0f);
        prey.setVariable<float>("type", 1.0f);
        prey.setVariable<int>("life", preyLifeDist(gen));
    }

    // Initialise grass agents
    int numGrass = initialPops.grassPop;
    AgentPopulation grassPopulation(model.Agent("grass"), numGrass);
    for (int i = 0; i < numGrass; i++) {
        AgentInstance grass = grassPopulation.getNextInstance();
        grass.setVariable<int>("id", i);
        grass.setVariable<float>("x", floatDist(gen));
        grass.setVariable<float>("y", floatDist(gen));
        grass.setVariable<float>("type", 2.0f);
        grass.setVariable<int>("dead_cycles", 0);
        grass.setVariable<int>("available", 1);

    }

    cuda_model.setPopulationData(grassPopulation);
    cuda_model.setPopulationData(predatorPopulation);
    cuda_model.setPopulationData(preyPopulation);


    /**
     * Execution
     */
    printf("Model initialised, beginning simulation...\n");
    cuda_model.simulate();
    printf("Simulation complete\n");

    return 0;
}

