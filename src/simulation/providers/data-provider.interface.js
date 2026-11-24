/**
 * GARUDA VISION - Base Data Provider Interface
 * 
 * Abstract base class defining the contract for all telemetry data providers
 * (SyntheticDataProvider, CarlaDataProvider, etc.).
 * Guarantees lifecycle state management and contract compliance.
 */

export class DataProvider {
  /**
   * @param {string} name - Identifier of the data provider
   */
  constructor(name = 'BaseDataProvider') {
    if (new.target === DataProvider) {
      throw new TypeError('Cannot construct DataProvider instances directly. Use a concrete subclass.');
    }
    this.name = name;
    this.isInitialized = false;
    this.isRunning = false;
    this.isPaused = false;
    this.noiseEnabled = false;
    this.sequenceNumber = 0;
    this.simulationTick = 0;
    this.updateRateHz = 20;
  }

  /**
   * Initializes the provider and loads resources.
   * @returns {Promise<void> | void}
   */
  async init() {
    throw new Error(`Method "init()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Starts or resumes telemetry generation.
   */
  start() {
    throw new Error(`Method "start()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Pauses telemetry generation.
   */
  pause() {
    throw new Error(`Method "pause()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Resumes paused telemetry generation.
   */
  resume() {
    throw new Error(`Method "resume()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Resets the simulation state to the beginning.
   */
  reset() {
    throw new Error(`Method "reset()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Advances simulation state by time step dt (in seconds).
   * @param {number} dt 
   */
  step(dt) {
    throw new Error(`Method "step(dt)" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Emits the next standardized telemetry payload strictly adhering
   * to docs/TELEMETRY_SCHEMA.md.
   * @returns {Object}
   */
  getNextTelemetry() {
    throw new Error(`Method "getNextTelemetry()" must be implemented by ${this.constructor.name}`);
  }

  /**
   * Enables or disables realistic sensor measurement noise.
   * @param {boolean} enabled 
   */
  setNoiseEnabled(enabled) {
    this.noiseEnabled = Boolean(enabled);
  }

  /**
   * Returns current noise status.
   * @returns {boolean}
   */
  isNoiseEnabled() {
    return this.noiseEnabled;
  }

  /**
   * Configures target update frequency.
   * @param {number} rateHz 
   */
  setUpdateRate(rateHz) {
    if (typeof rateHz === 'number' && rateHz >= 1 && rateHz <= 100) {
      this.updateRateHz = rateHz;
    }
  }

  /**
   * Returns metadata and health information about this provider.
   * @returns {{
   *   name: string,
   *   type: string,
   *   isInitialized: boolean,
   *   isRunning: boolean,
   *   isPaused: boolean,
   *   updateRateHz: number,
   *   noiseEnabled: boolean,
   *   status: string,
   *   details?: Object
   * }}
   */
  getProviderInfo() {
    throw new Error(`Method "getProviderInfo()" must be implemented by ${this.constructor.name}`);
  }
}
