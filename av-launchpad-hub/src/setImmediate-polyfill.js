// setImmediate polyfill for browser environments
// This is needed for Node.js packages like rosbag that expect setImmediate

let nextHandle = 1;
const tasksByHandle = {};
let currentlyRunningATask = false;

const setImmediate = function (callback) {
  if (typeof callback !== "function") {
    throw new TypeError("Callback must be a function");
  }

  const args = Array.prototype.slice.call(arguments, 1);
  const handle = nextHandle++;

  tasksByHandle[handle] = {
    callback: callback,
    args: args,
  };

  // Use MessageChannel for better performance, fallback to setTimeout
  if (typeof MessageChannel !== "undefined") {
    const channel = new MessageChannel();
    channel.port2.onmessage = function () {
      runIfPresent(handle);
    };
    channel.port1.postMessage(null);
  } else {
    setTimeout(function () {
      runIfPresent(handle);
    }, 0);
  }

  return handle;
};

const clearImmediate = function (handle) {
  delete tasksByHandle[handle];
};

function runIfPresent(handle) {
  if (currentlyRunningATask) {
    setTimeout(function () {
      runIfPresent(handle);
    }, 0);
  } else {
    const task = tasksByHandle[handle];
    if (task) {
      currentlyRunningATask = true;
      try {
        task.callback.apply(null, task.args);
      } finally {
        clearImmediate(handle);
        currentlyRunningATask = false;
      }
    }
  }
}

// Add to global scope
if (typeof globalThis !== "undefined") {
  globalThis.setImmediate = setImmediate;
  globalThis.clearImmediate = clearImmediate;
} else if (typeof window !== "undefined") {
  window.setImmediate = setImmediate;
  window.clearImmediate = clearImmediate;
} else if (typeof global !== "undefined") {
  global.setImmediate = setImmediate;
  global.clearImmediate = clearImmediate;
}

export { clearImmediate, setImmediate };
export default setImmediate;
