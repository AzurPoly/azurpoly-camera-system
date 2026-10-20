import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { viteSingleFile } from "vite-plugin-singlefile";

// AZP-Camera.exe embeds the interface as one HTML resource: scripts, styles and images are inlined.
export default defineConfig({
  plugins: [react(), viteSingleFile()],
  base: "./",
  server: {
    host: "127.0.0.1",
    port: 5173,
    strictPort: true,
    // The branding images live in ../assets, outside the Vite root.
    fs: { allow: [".."] },
  },
  build: { outDir: "dist", emptyOutDir: true },
});
