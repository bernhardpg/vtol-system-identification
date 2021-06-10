function [t_seq_m, y_seq_m, input_seq_m] = extract_man_data_full(i, maneuver_indices, t_seq, y_seq, input_seq)
    m_start = maneuver_indices(i);
    m_end = maneuver_indices(i + 1) - 1;

    % Get data sequence for this maneuver
    t_seq_m = t_seq(m_start:m_end);
    y_seq_m = y_seq(m_start:m_end,:);
    input_seq_m = input_seq(m_start:m_end,:);
end