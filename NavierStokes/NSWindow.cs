using System;
using System.Threading;
using System.Drawing;

using OpenTK;
using OpenTK.Graphics.OpenGL;

namespace gfx
{
    class NSWindow : GameWindow
    {
        public NSWindow() : base(600, 600) { }

        // Determine the size of the simulation
        const int N = 100;
        const int WIDTH = N + 2;
        const int HEIGHT = N + 2;

        // Simulation constants
        const float SOURCE = 0.5f;
        const float VELOCITY = 2f;
        const float VISC = 0.0001f;
        const float DIFF = 0.0001f;

        // Velocity and density arrays
        float[,] u = new float[WIDTH, HEIGHT];
        float[,] v = new float[WIDTH, HEIGHT];
        float[,] u_prev = new float[WIDTH, HEIGHT];
        float[,] v_prev = new float[WIDTH, HEIGHT];
        float[,] dens = new float[WIDTH, HEIGHT];
        float[,] dens_prev = new float[WIDTH, HEIGHT];

        // Last mouse position
        int lastMouseX;
        int lastMouseY;

        // Whether we are displaying velocity
        bool displayVelocity = false;

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            GL.ClearColor(Color.Black);

            // Enable blending for force drawing
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);

            this.MouseDown += NSWindow_MouseDown;
            this.MouseMove += NSWindow_MouseMove;
            this.KeyDown   += NSWindow_KeyDown;
        }

        private void NSWindow_KeyDown(object sender, OpenTK.Input.KeyboardKeyEventArgs e)
        {
            if (e.Key == OpenTK.Input.Key.V)
            {
                displayVelocity = !displayVelocity;
            }
        }

        private void NSWindow_MouseDown(object sender, OpenTK.Input.MouseButtonEventArgs e)
        {
            var coords = VoxelCoordFromView(e.X, e.Y);
            var i = coords.Item1;
            var j = coords.Item2;

            if (e.Mouse.IsButtonDown(OpenTK.Input.MouseButton.Right))
            {
                dens[i, j] += SOURCE;
            }
        }

        private void NSWindow_MouseMove(object sender, OpenTK.Input.MouseMoveEventArgs e)
        {
            var coords = VoxelCoordFromView(e.X, e.Y);
            var i = coords.Item1;
            var j = coords.Item2;

            if (i > 1 && i < N && j > 1 && j < N)
            {
                if (e.Mouse.IsButtonDown(OpenTK.Input.MouseButton.Right))
                {
                    dens[i, j] += SOURCE;
                }
                if (e.Mouse.IsButtonDown(OpenTK.Input.MouseButton.Left))
                {
                    // Add velocity in a line based on last mouse position
                    u[i, j] += VELOCITY * (e.X - lastMouseX);
                    v[i, j] += VELOCITY * (lastMouseY - e.Y);
                }
            }

            lastMouseX = e.X;
            lastMouseY = e.Y;
        }

        private Tuple<int, int> VoxelCoordFromView(int X, int Y)
        {
            var i = (int)((X / (float)Width) * N + 1);
            var j = (int)(((Height - Y) / (float)Height) * N + 1);

            return new Tuple<int, int>(i, j);
        }

        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);

            GL.Viewport(0, 0, Width, Height);

            double aspect_ratio = Width / (double)Height;

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            GL.Ortho(0.0, 1.0, 0.0, 1.0, 0, 1);
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);

            Solver.vel_step(N, ref u, ref v, ref u_prev, ref v_prev, VISC, .01f);
            Solver.dens_step(N, ref dens, ref dens_prev, ref u, ref v, DIFF, .01f);

            var keyboard = OpenTK.Input.Keyboard.GetState();
            if (keyboard[OpenTK.Input.Key.Escape])
            {
                Exit();
                return;
            } 
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            DrawDensity();

            if (displayVelocity)
            {
                DrawVelocity();
            }

            SwapBuffers();
            Thread.Sleep(1);
        }

        private void DrawDensity()
        {
            float x, y;
            float h = 1.0f / N;

            GL.Begin(PrimitiveType.Quads);

            for (var i = 0; i <= N; i++)
            {
                x = (i - 0.5f) * h;
                for (var j = 0; j <= N; j++)
                {
                    y = (j - 0.5f) * h;

                    var density = dens[i, j];
                    GL.Color3(density, density, density);
                    GL.Vertex2(x, y);
                    GL.Vertex2(x + h, y);
                    GL.Vertex2(x + h, y + h);
                    GL.Vertex2(x, y + h);
                }
            }

            GL.End();
        }

        private void DrawVelocity()
        {
            float x, y;
            float h = 1.0f / N;

            GL.Color4(0.9f, 0.0f, 0.0f, 0.35f);
            GL.LineWidth(1.0f);

            GL.Begin(PrimitiveType.Lines);

            for (var i = 1; i <= N; i++)
            {
                x = (i - 0.5f) * h;
                for (var j = 1; j <= N; j++)
                {
                    y = (j - 0.5f) * h;

                    GL.Vertex2(x, y);
                    GL.Vertex2(x + 0.1*u[i, j], y + 0.1*v[i, j]);
                }
            }

            GL.End();
        }
    }
}
