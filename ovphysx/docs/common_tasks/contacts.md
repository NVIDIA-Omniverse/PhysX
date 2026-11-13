<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Contacts

Contacts tell your program when objects touch and how hard they push on each other, such as a
robot's foot on the floor or a gripper's fingers on a part. ovphysx reports them through the same
read calls as positions.

![A 2 kg box falls onto a static plate. Next to it, a graph shows the contact force on the box. The force stays at zero during the fall and jumps to 488 N for one step when the box lands. Then it settles on a dashed line at the box's weight, 2 kg times 9.81 m/s², which is 19.62 N.](images/guide_contacts_force_graph.png)

## Contact Sensors

A contact sensor is a body that reports its contacts. The program in [Read the Contact Force](#read-the-contact-force) uses three scene
settings:

| Scene Setting | Why |
| --- | --- |
| `PhysxContactReportAPI` on the body, or on the prim that holds an [articulation](../building_scenes/articulations.md) | Makes the body, or every link of the articulation, a contact sensor. A body without it has no contact data. |
| `physxContactReport:threshold = 0` on the same prim | Reports even small forces. By default, contact forces below 1 N are not reported, in a scene in meters. |
| `physxScene:disableSleeping = 1` on the `PhysicsScene`, with `PhysxSceneAPI` | Keeps bodies awake. A body that lies still falls asleep after a moment, and then its contacts with sleeping or static objects are no longer reported. |

## Read the Contact Force

This program, `contacts_box_on_plate.py`, drops a 2 kg box onto a static plate and prints the contact force on the box every
5 steps:

```{literalinclude} code/contacts_box_on_plate.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/contacts_box_on_plate.out
:language: output
```

Notice the following:

- A read of `SimObjectType.CONTACT_SENSOR` returns one row per sensor. `netNormalContactForce`
  adds up the forces that press on the sensor where it touches other objects, as an x, y, z
  vector in newtons, in world coordinates.
- The force is the one that the sensor receives. The plate pushes the box up, so z is positive.
- Each value describes the last step, so a read returns zero before the first step.
- Friction forces, along the touching surfaces, come separately as `netFrictionContactForce`.
  Add the two vectors for the total contact force.

## Forces From One Object

The net force adds up the forces from every object that touches the sensor. To see what one object contributes,
name it as a filter: pass `contacts=[ContactSelection("/World/Box", ["/World/Plate"])]` to a
`read_sync` of `SimObjectType.CONTACT_SENSOR`, and read `normalContactForceMatrix`. It holds an
x, y, z force for each filter object, in the order of the list. `ContactSelection` comes from
`ovphysx.types`.

In the following run, a 1 kg box drops onto the 2 kg box, and the read uses two filters, the plate
and the top box:

![The 2 kg box rests on the plate while a 1 kg box drops onto it. Three bars show the read result. The net force on the 2 kg box stays near 19.62 N, while the force from the plate rises to 29.43 N. The force from the top box settles at -9.81 N, after a spike of about 200 N each way at the impact.](images/guide_contacts_filters.png)

At the impact, the plate and the top box each push about 200 N, in opposite directions, yet the
net force barely changes. Only the filters show that.

## More Detail

- [Contact Sensors](../read_write/readable.md#contact-sensors) lists every contact attribute,
  including contact points and contact events.
- [Contact Queries](../read_write/index.md#contact-queries) explains filters in full.
- [Contact Sensor Attributes](../read_write/attributes/contact_sensor.md) gives the layout of
  each attribute.
- [PhysxContactReportAPI](../population/PhysxContactReportAPI.md) describes the schema and its
  settings.

Next: [Showing the Simulation](showing_the_simulation.md)
