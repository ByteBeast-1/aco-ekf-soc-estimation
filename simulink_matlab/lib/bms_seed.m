function s = bms_seed(seed)
%BMS_SEED Convert a user seed into a valid (non-zero) xorshift32 state.
    s = bitxor(uint32(abs(seed)), uint32(2463534242));
    if s == 0
        s = uint32(1);
    end
end
