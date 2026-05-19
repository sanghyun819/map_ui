import { defineConfig } from "vite";

export default defineConfig({
  base: "./",
  server: {
    watch: {
      usePolling: process.env.VITE_USE_POLLING === "1",
      interval: 1000,
      ignored: ["**/.git/**", "**/node_modules/**", "**/dist/**", "**/release/**"],
    },
  },
});
