file(REMOVE mySolution.sol)
file(REMOVE mySolition.sol.PG.csv)

execute_process(
        COMMAND ./hgs ../Instances/CVRP/${INSTANCE}.vrp mySolution.sol -seed 1 -round ${ROUND} -useDecomposition 1
)

file(STRINGS mySolution.sol output REGEX "^Cost.*$")
list(GET output 0 line)
string(REGEX MATCH "Cost ([0-9]+)" cost line)

if(cost GREATER ${MAX_COST})
    message(SEND_ERROR "Test error for instance ${INSTANCE}. Cost ${cost} is too large (> ${MAX_COST})!")
endif()
