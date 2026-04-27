module.exports = async function beforeBuild() {
  // Renderer dependencies are bundled by Vite, and Electron main uses no runtime npm modules.
  return false;
};
