"""Per-marker checks as diagnostic_msgs/DiagnosticArray (the localizer's ~/markers).

One status per marker in view, named `marker_localizer: marker <id>`, from a
fix record (core/check.py fix_record with marker_views):

  level    OK, or WARN for a marker left out of the fix as an outlier
  message  used / outlier (why) / unused / not usable (why) / not in the map
  values   state, and when known residual_px, seen_x, seen_y, seen_z, seen_from,
           offset_m, angle_deg (core/check.py marker_views)
"""
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

NAME = 'marker_localizer: marker {}'
_KEYS = ('residual_px', 'seen_from', 'offset_m', 'angle_deg')


def marker_diagnostics(rec: dict, stamp) -> DiagnosticArray:
    out = DiagnosticArray()
    out.header.stamp = stamp
    for key, m in sorted((rec.get('markers') or {}).items(), key=lambda kv: int(kv[0])):
        st = DiagnosticStatus()
        st.name = NAME.format(key)
        state = m.get('state', '')
        st.level = DiagnosticStatus.WARN if state == 'outlier' else DiagnosticStatus.OK
        st.message = {'used': 'used', 'unused': 'unused', 'unknown': 'not in the map',
                      'outlier': f'outlier: {m.get("why", "")}',
                      'unusable': f'not usable: {m.get("why", "")}'}.get(state, state)
        vals = [('state', state)]
        vals += [(k, m[k]) for k in _KEYS if k in m]
        if 'seen' in m:
            vals += [(f'seen_{a}', m['seen'][a]) for a in ('x', 'y', 'z')]
        st.values = [KeyValue(key=k, value=str(v)) for k, v in vals]
        out.status.append(st)
    return out
