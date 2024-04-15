class SST:  # We will be using the drone model from the exam
    def __init__(
        self, start_state=None, goal_state=None, u_bounds_list=None, StateSpace=None
    ):
        self.start_state = start_state
        self.goal_state = goal_state
        self.Witness_Regions = self.WitnessRegions()
        self.Active_Set = self.ActiveSet()
        self.Inactive_Set = self.InactiveSet()
        # self.Active_Set.Has_Witness = False
        self.StateSpace = StateSpace
        None

    def SST(X, U, x_0, T_prop, N, delta_BestNeighbor, delta_s):
        None

    def Is_Node_Locally_Best_SST(X_new, S, delta_s):
        None

    def Prune_Dominated_Nodes_SST(X_new, V_active, V_inactive, E):
        None

    def Best_First_Selection_SST(X, V, delta_BestNeighbor):
        x_rand = StateValidation.StateSample(
            boundingMesh, pitch_bounds=pitch_bounds, velocity_bounds=velocity_bounds
        )
        x_nearList = [
            (Get3Ddist(V[i][0:3], x_rand[0:3]), i)
            for i in range(0, len(V))
            if Get3Ddist(V[i][0:3], x_rand[0:3]) < delta_BestNeighbor
        ]
        x_near = [
            V[i]
            for i in range(0, len(V))
            if Get3Ddist(V[i][0:3], x_rand[0:3]) < delta_BestNeighbor
        ]
        if len(x_near) == 0:  # no neighbors within delta_BestNeighbor
            None

    def MonteCarlo_Prop(x_selected, U, T_prop):
        # x_selected is chosen neighbor, U is input bounds dict, T is max propagation time
        numInputs = len(U)
        random_inputs = np.empty([1, numInputs])
        for input in range(0, numInputs):
            inputBounds = input.value  # get bounds of input from dictionary
            inputSample = random.uniform(inputBounds[0], inputBounds[1])
            random_inputs[0][input] = random.uniform(U[input][0], U[input][1])
        t = random.uniform(0, T_prop)

    def Randomized_Inputs(u_bounds_list):
        random_inputs = np.empty([1, len(u_bounds_list)])
        numInputs = len(u_bounds_list)

        for i in range(0, numInputs):
            random_inputs[0][i] = random.uniform(
                u_bounds_list[i][0], u_bounds_list[i][1]
            )
        return random_inputs

    class ActiveSet:
        def __init__(self):
            self.States = []
            self.States.append(self.start_state)
            None

    class InactiveSet:
        def __init__(self):
            None

    class WitnessRegions:
        def __init__(self, x_0, d_s, cost):
            self.Regions = []
            region = self.Region(x_0, d_s, cost)
            self.Regions.append(region)

        def AddNewRegion(self, rep, d_s, cost):
            region = self.Region(rep, d_s, cost)
            self.Regions.append(region)

        class Region:
            def __init__(self, rep, d_s, cost):
                self.representative = rep  # current state that represents the region
                x = rep.state[0]
                y = rep.state[1]
                z = rep.state[2]
                self.cost = cost
                regionSphere = trimesh.creation.icosphere(3, d_s)
                regionSphere.apply_translation([x, y, z])
                self.regionMesh = regionSphere

            def ChangeRepresentative(self, x, cost):
                self.representative = x
                self.cost = cost
