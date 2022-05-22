using CSV
using DataFrames
using Distributions
using LaTeXStrings
using LinearAlgebra
using Plots
using Rotations
using ReferenceFrameRotations

N = 500
dt = 0.1
tend = 100.0

noise_Σ = [0.00174533^2 0.0        0.0
           0.0        0.00174533^2 0.0
           0.0        0.0        0.00174533^2]
noise = MvNormal(noise_Σ)

t = Vector{Float64}(undef, N)
t[1] = 0.0

ω₁ = Vector{Float64}(undef, N)
ω₂ = Vector{Float64}(undef, N)
ω₃ = Vector{Float64}(undef, N)
ω₁[1] = 0.0
ω₂[1] = 0.0
ω₃[1] = 0.0

q₀ = Vector{Float64}(undef, N)
q₁ = Vector{Float64}(undef, N)
q₂ = Vector{Float64}(undef, N)
q₃ = Vector{Float64}(undef, N)
q₀[1] = 1.0
q₁[1] = 0.0
q₂[1] = 0.0
q₃[1] = 0.0

roll = Vector{Float64}(undef, N)
pitch = Vector{Float64}(undef, N)
yaw = Vector{Float64}(undef, N)
roll[1] = 0.0
pitch[1] = 0.0
yaw[1] = 0.0

ωw₁ = Vector{Float64}(undef, N)
ωw₂ = Vector{Float64}(undef, N)
ωw₃ = Vector{Float64}(undef, N)
ωw₁[1] = 0.0
ωw₂[1] = 0.0
ωw₃[1] = 0.0

qw₀ = Vector{Float64}(undef, N)
qw₁ = Vector{Float64}(undef, N)
qw₂ = Vector{Float64}(undef, N)
qw₃ = Vector{Float64}(undef, N)
qw₀[1] = 1.0
qw₁[1] = 0.0
qw₂[1] = 0.0
qw₃[1] = 0.0

rollw = Vector{Float64}(undef, N)
pitchw = Vector{Float64}(undef, N)
yaww = Vector{Float64}(undef, N)
rollw[1] = 0.0
pitchw[1] = 0.0
yaww[1] = 0.0

I = [2500.0 0.0    0.0
     0.0    2300.0 0.0
     0.0    0.0    3000.0]

ω_o = 2.0 * π / 5926.0

for k ∈ range(1, N - 1)
    # System
    ω = [ω₁[k]; ω₂[k]; ω₃[k]]
    q = [q₀[k]; q₁[k]; q₂[k]; q₃[k]]

    euler = RotXYZ(QuatRotation(q₀[k], q₁[k], q₂[k], q₃[k]))
    roll[k] = euler.theta1 * 57.2958
    pitch[k] = euler.theta2 * 57.2958
    yaw[k] = euler.theta3 * 57.2958

    # Noise
    ωw₁[k] = ω₁[k] + 0.00174533
    ωw₂[k] = ω₂[k] - 0.00174533
    ωw₃[k] = ω₃[k] + 0.002617994

    w = rand(noise, 1)
    aw = norm(w)
    ew = w / aw * sin(aw / 2)
    qw = QuatRotation(cos(aw / 2), ew[1], ew[2], ew[3])
    qw = qw / norm(qw)
    qw = QuatRotation(q₀[k], q₁[k], q₂[k], q₃[k]) * qw

    qw₀[k] = qw[1]
    qw₁[k] = qw[2]
    qw₂[k] = qw[3]
    qw₃[k] = qw[4]

    eulerw = RotXYZ(qw)
    rollw[k] = eulerw.theta1 * 57.2958
    pitchw[k] = eulerw.theta2 * 57.2958
    yaww[k] = eulerw.theta3 * 57.2958

    # Dynamics
    R_bo = [(1.0 - 2.0 * (q₂[k]^2.0 + q₃[k]^2.0))   (2.0 * (q₁[k] * q₂[k] + q₃[k] * q₀[k])) (2.0 * (q₁[k] * q₃[k] - q₂[k] * q₀[k]))
            (2.0 * (q₂[k] * q₁[k] - q₃[k] * q₀[k])) (1.0 - 2.0 * (q₁[k]^2.0 + q₃[k]^2.0))   (2.0 * (q₂[k] * q₃[k] - q₁[k] * q₀[k]))
            (2.0 * (q₃[k] * q₁[k] - q₂[k] * q₀[k])) (2.0 * (q₃[k] * q₂[k] - q₁[k] * q₀[k])) (1.0 - 2.0 * (q₁[k]^2.0 + q₂[k]^2.0))]

    ω_bo = ω - ω_o * R_bo[:, 2]
    ω_bo₁ = ω_bo[1]
    ω_bo₂ = ω_bo[2]
    ω_bo₃ = ω_bo[3]

    M_gg = 3.0 * ω_o^2.0 * cross(R_bo[:, 3], I * R_bo[:, 3])

    M_d = [0.001; 0.001; 0.001]

    dω = inv(I) * (M_gg + M_d - cross(ω, I * ω)) * dt

    dq = 0.5 * [ 0.0    ω_bo₃ -ω_bo₂ ω_bo₁
                -ω_bo₃  0.0    ω_bo₁ ω_bo₂
                 ω_bo₂ -ω_bo₁  0.0   ω_bo₃
                -ω_bo₁ -ω_bo₂ -ω_bo₃ 0.0] * q * dt 

    t[k + 1] = t[k] + dt

    ω₁[k + 1] = ω₁[k] + dω[1]
    ω₂[k + 1] = ω₂[k] + dω[2]
    ω₃[k + 1] = ω₃[k] + dω[3]

    q₀[k + 1] = q₀[k] + dq[1]
    q₁[k + 1] = q₁[k] + dq[2]
    q₂[k + 1] = q₂[k] + dq[3]
    q₃[k + 1] = q₃[k] + dq[4]
end

df = DataFrame(t = t,
omega1 = ω₁ * 0.0174533,
omega2 = ω₂ * 0.0174533,
omega3 = ω₃ * 0.0174533,
q0 = q₀,
q1 = q₁,
q2 = q₂,
q3 = q₃,
roll = roll,
pitch = pitch,
yaw = yaw,
omegaw1 = ωw₁ * 0.0174533,
omegaw2 = ωw₂ * 0.0174533,
omegaw3 = ωw₃ * 0.0174533,
qw0 = qw₀,
qw1 = qw₁,
qw2 = qw₂,
qw3 = qw₃,
rollw = rollw,
pitchw = pitchw,
yaww = yaww)

df = df[1:end - 1, :]

# Plot
p1 = plot(df.t, [df.roll, df.pitch, df.yaw, df.rollw, df.pitchw, df.yaww], ylabel = L"Attitude $(\,\circ)$")
p1[1][1][:label] = L"\phi"
p1[1][2][:label] = L"\theta"
p1[1][3][:label] = L"\psi"
p1[1][4][:label] = L"\phi_w"
p1[1][5][:label] = L"\theta_w"
p1[1][6][:label] = L"\psi_w"

p2 = plot(df.t, [df.omega1, df.omega2, df.omega3, df.omegaw1, df.omegaw2, df.omegaw3], xlabel = L"t (\textrm{s})", ylabel = L"Angular Rate $(\,\circ/\textrm{s})$")
p2[1][1][:label] = L"\phi"
p2[1][2][:label] = L"\theta"
p2[1][3][:label] = L"\psi"
p2[1][4][:label] = L"\phi_w"
p2[1][5][:label] = L"\theta_w"
p2[1][6][:label] = L"\psi_w"

plot(p1, p2, layout = (2, 1))
savefig("plot.png")

# Output to CSV
CSV.write("output.csv", df)