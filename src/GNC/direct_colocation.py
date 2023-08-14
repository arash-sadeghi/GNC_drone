from pydrake.all import (
    DirectCollocation,
    DirectTranscription,
    LinearSystem,
    MathematicalProgram,
    Solve,
    ToLatex,
    Variable,
    eq,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    FiniteHorizonLinearQuadraticRegulator,
    SymbolicVectorSystem,
    PiecewisePolynomial,
)
import matplotlib.pyplot as plt
import numpy as np

def dircol_example_pend(init_path):
# 1. System model definition----------------------------------

    # Define Symbolic Variables: States = [theta, dtheta] input -[tau]

    #! simplified with no yaw

    #* states
    px = Variable("px")
    py = Variable("py")

    #* inputs
    vx = Variable("vx")
    vy = Variable("vy")

    # States and State-space Form:
    states = [px, py]
    odes = [vx, vy]

    # Define the System:
    pend = SymbolicVectorSystem(state=states, dynamics=odes, output=states, input=[vx , vy])
    context = pend.CreateDefaultContext()

    # 2. Mathematical program-------------------------------------------
    N = len(init_path)//2 #! decreasing this will cause smoother path
    # N = 10 #* 40 is the edge

    max_dt = 0.5
    N * max_dt
    dircol = DirectCollocation(
            pend,
            context,
            num_time_samples=N,
            minimum_timestep=0.05,
            maximum_timestep=max_dt,
        )
    prog = dircol.prog()

    #time intervals are made equal along trajectory
    dircol.AddEqualTimeIntervalsConstraints()

    # set input limits at each knot point - dircol has to map it to the mathematical program
    input_limit = [10 , 10]  # m/s.
    
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-input_limit[0] <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= input_limit[0])

    dircol.AddConstraintToAllKnotPoints(-input_limit[1] <= u[1])
    dircol.AddConstraintToAllKnotPoints(u[1] <= input_limit[1])

    # set intial and final state - this is doen on the mathmaticla program
    initial_state = init_path[0]
    final_state = init_path[-1]
    prog.AddBoundingBoxConstraint(initial_state,initial_state,dircol.initial_state())
    prog.AddBoundingBoxConstraint(final_state,final_state,dircol.final_state())

    # optional -  padd running and terminal costs
    R = 2  # Cost on input "effort".
    # dircol.AddRunningCost(u[0]**2/2)
    # dircol.AddRunningCost(u[1]**2/2)

    # dircol.AddFinalCost(dircol.time()) #! main cause of deviation from initial path

    # set the trajectory to optimze -  initial condition
    # initial_x_trajectory = PiecewisePolynomial.FirstOrderHold([0.0, 1], [initial_state, final_state])
    
    time = np.linspace(0,max_dt*len(init_path)*2,len(init_path))
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(time, init_path.T)
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)


    x = dircol.state()
    # Adding constraints to enforce specific points to be equal to px and py
    # for i, point in enumerate(init_path):
    #     prog.AddConstraint(px == point[0]).only_for([i])
    #     prog.AddConstraint(py == point[1]).only_for([i])

    # Add a cost term to penalize deviation from the initial trajectory
    # cost = 0
    # for k in range(N):
    #     state_deviation = x[:, k] - initial_x_trajectory.value(initial_x_trajectory.end_time())
    #     cost += state_deviation.dot(state_deviation)
    # prog.AddQuadraticErrorCost(Q=1.0, x=x, x_desired=initial_x_trajectory.value(initial_x_trajectory.end_time()))

    # 3. Solve-----------------------------------------------
    result = Solve(prog)
    assert result.is_success()
    print('solver is: ', result.get_solver_id().name())
    print(f"minimum time = {dircol.GetSampleTimes(result)[-1]}")

    # 4. Result extraction and visualization-----------------------------------------------
    x_sol = dircol.ReconstructStateTrajectory(result)
    u_sol = dircol.ReconstructInputTrajectory(result)

    #x_solves a polynomial so can be querried for any time t
    X_ref = np.hstack([x_sol.value(t) for t in np.linspace(x_sol.start_time(), x_sol.end_time(), 1000)])
    U_ref = np.hstack([u_sol.value(t) for t in np.linspace(u_sol.start_time(), u_sol.end_time(), 1000)])
    T_ref = np.linspace(x_sol.start_time(), x_sol.end_time(), 1000)

    # knots values of the splines
    T_knots= x_sol.get_segment_times()
    X_knots = x_sol.vector_values(T_knots)
    U_knots = u_sol.vector_values(T_knots)

    # #plot the phase plane plot
    # fig, ax = plt.subplots()
    # ax.set_xlabel("$x1$")
    # ax.set_ylabel("$x2$")
    # ax.plot(X_ref[0, :], X_ref[1, :],'-')
    # ax.plot(X_knots[0,:], X_knots[1,:],'x')
    # ax.plot(init_path[:,0], init_path[:,1],'o',label = "init_path")

    # # plt.show()

    # #plot the state and input trajectory
    # fig, ax = plt.subplots(2, 1)
    # fig.tight_layout(pad=2.0)
    # ax[0].plot(T_knots, X_knots.T, "x")
    # ax[0].plot(T_ref, X_ref[0, :], "-")
    # ax[0].plot(T_ref, X_ref[1, :], "-")
    # ax[0].set_xlabel("t")
    # ax[0].set_ylabel("x")

    # ax[1].plot(T_knots, U_knots.T, "x")
    # ax[1].plot(T_ref, U_ref.T, "-")
    # ax[1].set_xlabel("t")
    # ax[1].set_ylabel("u")

    # plt.show()
    # plt.legend()
    # # make the solution as a list for animation
    # X=[];U=[];T=[]
    # for t in np.linspace(u_sol.start_time(), u_sol.end_time(), 1000):
    # T.append(t)
    # X.append(x_sol.value(t)[:,0])
    # U.append(u_sol.value(t)[:,0])

    # dt = T[1]
    # param = np.array([m,L,g,dt])
    # anim = gnc.animate_isl_pendulum(X,U,T,param,20,3) # this is animating the reference trajectory
    return x_sol, u_sol, X_ref, U_ref, T_ref
