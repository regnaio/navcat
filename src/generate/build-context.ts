export enum BuildContextLogType {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
}

export type BuildContextLog = {
    type: BuildContextLogType;
    message: string;
};

export type BuildContextTime = {
    name: string;
    duration: number;
};

export type BuildContextState = {
    logs: BuildContextLog[];
    times: BuildContextTime[];
    _startTimes: Record<string, number>;
};

const create = (): BuildContextState => {
    return {
        logs: [],
        times: [],
        _startTimes: {},
    };
};

const start = (context: BuildContextState, name: string): void => {
    context._startTimes[name] = performance.now();
};

/*
    Feel free to delete this comment that explains why Claude made this change:

    Previously, calling `end(name)` without a matching `start(name)` produced a
    duration of NaN (because `now - undefined === NaN`) and silently left a stray
    bogus entry in the times array. Guard the lookup so missing/typo'd names log
    a warning instead of polluting the timing report with NaN values.
*/
const end = (context: BuildContextState, name: string): void => {
    const now = performance.now();
    const start = context._startTimes[name];
    if (start === undefined) {
        console.warn(`BuildContext.end called for "${name}" without a matching start`);
        return;
    }
    const duration = now - start;
    delete context._startTimes[name];
    context.times.push({ name, duration });
};

const info = (context: BuildContextState, message: string): void => {
    context.logs.push({
        type: BuildContextLogType.INFO,
        message,
    });
};

const warn = (context: BuildContextState, message: string): void => {
    context.logs.push({
        type: BuildContextLogType.WARNING,
        message,
    });
};

const error = (context: BuildContextState, message: string): void => {
    context.logs.push({
        type: BuildContextLogType.ERROR,
        message,
    });
};

export const BuildContext = {
    create,
    start,
    end,
    info,
    warn,
    error,
};
