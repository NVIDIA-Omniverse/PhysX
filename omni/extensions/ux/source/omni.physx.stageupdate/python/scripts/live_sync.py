# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
try:
    import omni.kit.usd.layers as layers
    import omni.timeline.live_session
except:
    pass


class StageUpdateLiveSync():
    def startup(self, stage_update_iface):
        self._stage_update_iface = stage_update_iface
        self._usd_context = None
        self._layers = None
        self._layers_event_subscription = None
        self._live_syncing = None
        self._timeline_session = None
        try:
            self._timeline_session = omni.timeline.live_session.get_timeline_session()
            usd_context_name: str = ""
            self._usd_context = omni.usd.get_context(usd_context_name)
            self._layers = layers.get_layers(self._usd_context)
            self._live_syncing = layers.get_live_syncing(self._usd_context)

            self._layers_event_subscription = self._layers.get_event_stream().create_subscription_to_pop_by_type(
                layers.LayerEventType.LIVE_SESSION_STATE_CHANGED, self._on_layers_event, name="OmniPhysX.StageUpdate"
            )
        except:  
            pass

    def _start_subscription(self):
        # Get the timeline session and register our own callback for presenter changed
        self._timeline_session = omni.timeline.live_session.get_timeline_session()
        if self._timeline_session is not None:
            self._timeline_session.add_presenter_changed_fn(self._on_presenter_changed)

    def _stop_subscription(self):
        if self._timeline_session is not None:
            self._timeline_session.remove_presenter_changed_fn(self._on_presenter_changed)

    def _on_layers_event(self, event):
        try:
            payload = layers.get_layer_event_payload(event)
            if not payload:
                return

            # Only events from root layer session are handled.
            if payload.event_type == layers.LayerEventType.LIVE_SESSION_STATE_CHANGED:
                if not payload.is_layer_influenced(self._usd_context.get_stage_url()):
                    return

                if not self._live_syncing.is_in_live_session():
                    self._stop_subscription()
                    self._stage_update_iface.block_timeline_events(False)
                else:
                    self._start_subscription()
                    if self._timeline_session.am_i_presenter():
                        self._stage_update_iface.block_timeline_events(False)
                    else:
                        self._stage_update_iface.block_timeline_events(True)
        except:
            pass

    def _on_presenter_changed(self, user):
        if self._timeline_session.am_i_presenter():
            self._stage_update_iface.block_timeline_events(False)
        else:
            self._stage_update_iface.block_timeline_events(True)

    def shutdown(self):
        self._stop_subscription()
        self._stage_update_iface = None
        self._usd_context = None
        self._layers = None
        self._layers_event_subscription = None
        self._live_syncing = None
