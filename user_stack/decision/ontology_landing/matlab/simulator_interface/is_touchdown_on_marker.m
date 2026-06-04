function isTouchdown = is_touchdown_on_marker(uavState, observation, cfg)
%IS_TOUCHDOWN_ON_MARKER Compatibility wrapper for pad-contact touchdown.

contact = detect_landing_contact(uavState, observation, cfg, true);
isTouchdown = contact.detected && contact.success && strcmp(contact.type, 'pad');
end
