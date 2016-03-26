using System;

namespace gfx
{
    public static class Solver
    {
        // Solver from J. Stam, Real-Time Fluid Dynamics for Games

        public static void diffuse(int N, int b, ref float[,] x, ref float[,] x0, float diff, float dt)
        {
            float a = dt * diff * N * N;

            for (var k = 0; k < 20; k++)
            {
                for (var i = 1; i <= N; i++)
                {
                    for (var j = 1; j <= N; j++)
                    {
                        x[i, j] = (x0[i, j] + a * (x[i - 1, j] + x[i + 1, j] + x[i, j - 1] + x[i, j + 1])) / (1 + 4 * a);
                    }
                }
                set_bnd(N, b, ref x);
            }
        }

        public static void advect(int N, int b, ref float[,] d, ref float[,] d0, ref float[,] u, ref float[,] v, float dt)
        {
            int i, j, i0, j0, i1, j1;
            float x, y, s0, t0, s1, t1, dt0;
            dt0 = dt * N;
            for (i = 1; i <= N; i++)
            {
                for (j = 1; j <= N; j++)
                {
                    x = i - dt0 * u[i, j];
                    y = j - dt0 * v[i, j];

                    if (x < 0.5)
                        x = 0.5F;
                    if (x > N + 0.5)
                        x = N + 0.5F;

                    i0 = (int)x;
                    i1 = i0 + 1;

                    if (y < 0.5)
                        y = 0.5F;
                    if (y > N + 0.5)
                        y = N + 0.5F;

                    j0 = (int)y;
                    j1 = j0 + 1;
                    s1 = x - i0;
                    s0 = 1 - s1;
                    t1 = y - j0;
                    t0 = 1 - t1;

                    d[i, j] = s0 * (t0 * d0[i0, j0] + t1 * d0[i0, j1]) + s1 * (t0 * d0[i1, j0] + t1 * d0[i1, j1]);
                }
            }
            set_bnd(N, b, ref d);
        }

        public static void dens_step(int N, ref float[,] x, ref float[,] x0, ref float[,] u, ref float[,] v, float diff, float dt)
        {
            diffuse(N, 0, ref x0, ref x, diff, dt);
            advect(N, 0, ref x, ref x0, ref u, ref v, dt);
        }        public static void vel_step(int N, ref float[,] u, ref float[,] v, ref float[,] u0, ref float[,] v0, float visc, float dt)
        {
            diffuse(N, 1, ref u0, ref u, visc, dt);

            diffuse(N, 2, ref v0, ref v, visc, dt);
            project(N, ref u0, ref v0, ref u, ref v);

            advect(N, 1, ref u, ref u0, ref u0, ref v0, dt);
            advect(N, 2, ref v, ref v0, ref u0, ref v0, dt);
            project(N, ref u, ref v, ref u0, ref v0);
        }        public static void project(int N, ref float[,] u, ref float[,] v, ref float[,] p, ref float[,] div)
        {
            int i, j, k;
            float h;
            h = 1.0F / N;
            for (i = 1; i <= N; i++)
            {
                for (j = 1; j <= N; j++)
                {
                    div[i, j] = -0.5F * h * (u[i + 1, j] - u[i - 1, j] + v[i, j + 1] - v[i, j - 1]);
                    p[i, j] = 0;
                }
            }
            set_bnd(N, 0, ref div); set_bnd(N, 0, ref p);
            for (k = 0; k < 20; k++)
            {
                for (i = 1; i <= N; i++)
                {
                    for (j = 1; j <= N; j++)
                    {
                        p[i, j] = (div[i, j] + p[i - 1, j] + p[i + 1, j] + p[i, j - 1] + p[i, j + 1]) / 4;
                    }
                }
                set_bnd(N, 0, ref p);
            }
            for (i = 1; i <= N; i++)
            {
                for (j = 1; j <= N; j++)
                {
                    u[i, j] -= 0.5F * (p[i + 1, j] - p[i - 1, j]) / h;
                    v[i, j] -= 0.5F * (p[i, j + 1] - p[i, j - 1]) / h;
                }
            }
            set_bnd(N, 1, ref u); set_bnd(N, 2, ref v);
        }

        private static void set_bnd(int N, int b, ref float[,] x)
        {
            for (var i = 1; i <= N; i++)
            {
                x[0, i] = b == 1 ? x[1, i] * -1 : x[1, i];
                x[N + 1, i] = b == 1 ? x[N, i] * -1 : x[N, i];
                x[i, 0] = b == 2 ? x[i, 1] * -1 : x[i, 1];
                x[i, N + 1] = b == 2 ? x[i, N] * -1 : x[i, N];
            }
            x[0, 0] = 0.5F * (x[1, 0] + x[0, 1]);
            x[0, N + 1] = 0.5F * (x[1, N + 1] + x[0, N]);
            x[N + 1, 0] = 0.5F * (x[N, 0] + x[N + 1, 1]);
            x[N + 1, N + 1] = 0.5F * (x[N, N + 1] + x[N + 1, N]);
        }
    }
}
