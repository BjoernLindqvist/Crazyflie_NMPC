# Same as second example, but using F1 (ALM)
import casadi.casadi as cs
import opengen as og
import json

nu = 3
np = 1
u = cs.SX.sym("u", nu)
p = cs.SX.sym("p", np)

f = cs.dot(u, u)
for i in range(nu):
    f += p * u[i]

F1 = cs.sin(u[0]) - 0.3
C = og.constraints.Zero()

U = og.constraints.Ball2(None, 0.5)

problem = og.builder.Problem(u, p, f)         \
    .with_constraints(U)                      \
    .with_aug_lagrangian_constraints(F1, C)

meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["Shane Trimble"])            \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("shane")

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_build")      \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()

solver_config = og.config.SolverConfiguration()   \
            .with_tolerance(1e-5)                 \
            .with_initial_tolerance(1e-5)         \
            .with_initial_penalty(10)             \
            .with_penalty_weight_update_factor(2) \
            .with_max_outer_iterations(20)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)
builder.build()

mng = og.tcp.OptimizerTcpManager('python_build/shane')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
solution = mng.call([1.0])  # call the solver over TCP
print(json.dumps(solution, indent=4, sort_keys=False))

mng.kill()
