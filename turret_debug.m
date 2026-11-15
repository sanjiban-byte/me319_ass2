function out = turret_debug(action, s)
%TURRET_DEBUG  Side channel for logging controller internals (harness only).
%   turret_debug('set', s) stores a struct; s = turret_debug('get') reads it.
persistent store
out = [];
if strcmp(action, 'set'), store = s; else, out = store; end
end
