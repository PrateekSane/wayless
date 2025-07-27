// Comprehensive Draco3D stub to handle all import variations
// This provides interfaces for both the main draco3d module and WASM decoder

// Create a mock decoder module
const createMockDecoder = () => ({
  DecoderBuffer: function () {
    return {
      Init: () => true,
      GetDataSize: () => 0,
    };
  },
  Decoder: function () {
    return {
      DecodeGeometry: () => null,
      GetAttribute: () => null,
      GetAttributeByUniqueId: () => null,
      delete: () => {},
    };
  },
  // Common Draco constants that might be expected
  POSITION_ATTRIBUTE: 0,
  NORMAL_ATTRIBUTE: 1,
  COLOR_ATTRIBUTE: 2,
  TEX_COORD_ATTRIBUTE: 3,
});

// Main draco module interface
const dracoModule = {
  createDecoderModule: () => Promise.resolve(createMockDecoder()),
  ...createMockDecoder(),
};

// Support both CommonJS and ES modules
if (typeof module !== "undefined" && module.exports) {
  module.exports = dracoModule;
  module.exports.default = dracoModule;
}

// ES module exports
export default dracoModule;
export const createDecoderModule = dracoModule.createDecoderModule;
export const DecoderBuffer = dracoModule.DecoderBuffer;
export const Decoder = dracoModule.Decoder;
export const POSITION_ATTRIBUTE = dracoModule.POSITION_ATTRIBUTE;
export const NORMAL_ATTRIBUTE = dracoModule.NORMAL_ATTRIBUTE;
export const COLOR_ATTRIBUTE = dracoModule.COLOR_ATTRIBUTE;
export const TEX_COORD_ATTRIBUTE = dracoModule.TEX_COORD_ATTRIBUTE;
