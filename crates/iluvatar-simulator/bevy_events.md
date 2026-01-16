
Events and messages let us decouple what happened from what should happen by allowing communication between systems.

There are two kind of events in Bevy:

    Message for communication between systems
    Event and EntityEvent for observers that trigger immediate behavior

Messages

A Message is the core part of Bevy's buffered queue style event system.

Each message is tracked individually using its MessageId. These IDs auto increment based on the order the messages were sent.

Systems can use the MessageReader system parameter to read events from the queue. Each reader contains a Local<MessageCursor> that holds the systems progress in reading from the Messages<T> resource. This is how each system ensures that it is not reading the same event twice.

MessageWriter is the system parameter we use to write messages to a queue. These are stored inside a Messages<T> resource which is basically just a wrapper around a Vec<T> with some convenient accessors.
Messages are double buffered

Messages<T> is a collection that acts as a double buffered queue.

pub struct Messages<E: Message> {
    /// Holds the oldest still active messages.
    pub(crate) messages_a: MessageSequence<E>,
    /// Holds the newer messages.
    pub(crate) messages_b: MessageSequence<E>,
    pub(crate) message_count: usize,
}

Double buffering is done to ensure each system has an opportunity to see each message. It is helping systems not have to care about the exact ordering within a frame.

To illustrate this double buffering, imagine you have a game with a system that publishes messages when the player gets detected with a PlayerDetected message.

fn main() {
  App::new()
    .add_plugins(DefaultPlugins)
    .add_systems(
      Update,
      (
        on_player_detected,
        detect_player.after(on_player_detected)
      )
    )
    .run()
}

When you start your game both buffers start out empty:

Buffer A (current): []
Buffer B (previous): []

Then we publish a message from detect_player using a MessageWriter, it pushes it to the buffer:

Buffer A (current): [PlayerDetected]
Buffer B (previous): []

Unfortunately detect_player runs after our reading system on_player_detected. If we only had one buffer and we wiped it then the on_player_detected system would never get to read the published message.

Instead, at the end of this game tick the oldest buffer (Buffer B) will be cleared and made to be our primary buffer.

Buffer A (previous): [PlayerDetected]
Buffer B (current): []

In our example, the on_player_detected that read the messages can use the original Buffer A holding the messages for the previous frame.

Even if more messages from other systems were published, the Buffer A would be holding the last frame's message.

Buffer A (previous): [PlayerDetected]
Buffer B (current): [PlayerDetected]

This lets the systems earlier in the frame catch up by reading the last frame's messages.

Finally at the end of the third tick the oldest buffer (Buffer A) is cleared and set to be our current buffer to write messages to, leaving our buffer looking like:

Buffer A (current): []
Buffer B (previous): [PlayerDetected]

In this way our systems have access to both this frame and the last frames messages. So it shouldn't matter how we order our systems to be before or after we fire our messages, it will have access to both.
Adding messages

Messages are defined just like our resources and components where we define a type that derives Message:

// With a marker message
#[derive(Message)]
struct PlayerKilled;

// With a unit type
#[derive(Message)]
struct PlayerDetected(Entity);

// With fields
#[derive(Message)]
struct PlayerDamaged {
  entity: Entity,
  damage: f32,
}

Then we add our message to our App, similar to how we manage our assets:

fn main() {
  App::new()
    .add_message::<PlayerKilled>();
}

When we add_message, Bevy adds a system for handling that specific type: Messages<T>::message_update_system.

This system runs each frame, cleaning up any unconsumed messages by calling Messages<T>::update. If this function were not called, then our messages will grow unbounded eventually exhausting the queue.

This also means that if your messages are not consumed by the next frame then they will be cleaned up and dropped silently.
Writing messages

Messages are written to a double buffered queue. This just means that messages produced are stored for two frames.

This prevents a situation where a system that was called earlier will miss the message, which is very common because Bevy is working hard to run our systems in parallel.

For example, if you had defined a system to only run conditionally, then its possible to miss messages during the frames it was not called.

To write messages to a stream we use a MessageWriter<T>. Any two systems that use the same message writer type will not be run in parallel as they both use mutable access to Messages<T>.

fn detect_player(
  mut messages: MessageWriter<PlayerDetected>,
  players: Query<(Entity, &Transform), With<Player>>,
) {
  for (entity, transform) in players {
    // ...
    messages.write(PlayerDetected(entity));
  }
}

Each MessageWriter<T> can only write messages for one type that is known during compile time. There may be times where you don't know this type and as a work around you can send type erased messages through your Commands

commands.queue(|world: &mut World| {
  world.write_message(MyMessage);
});

Reading messages

This double buffering strategy means that we must consume our messages steadily each frame or risk losing them. We can read messages from our systems with an MessageReader<T> that consumes messages from our buffers:

fn react_to_detection(mut messages: MessageReader<PlayerDetected>) {
  for message in messages.read() {
    // Do something with each event here
  }
}

A MessageReader system parameter tracks the consumption of these events on a per-system basis using a Local<MessageCursor>, which will guarantee each system an opportunity to read the event once.

If you have many different types of events you want handled the same way you can use a generic system and a Messages resource:

fn handle_message<T: Message>(mut messages: ResMut<Messages<T>>) {
  // We can clear messages this frame
  messages.clear();

  // Or clear messages next frame (bevy default)
  messages.update();

  // Or consume our messages right here and now
  for message in messages.drain() {
    // ...
  }
}

fn main() {
  App::new()
    .add_message::<PlayerKilled>()
    .init_resource::<Messages<PlayerKilled>>()
    .add_systems(Update, handle_message::<PlayerKilled>)
    .run();
}

So we can say that the Messages<T> resource represents a collection of all messages of the type that occurred in the last two update calls.
Events

Bevy implements Event as a trait with a Trigger type.

trait Event {
  type Trigger<'a>: Trigger<Self>;
}

We derive this trait on our own event structs:

#[derive(Event)]
struct GameStarted;

#[derive(EntityEvent)]
struct PlayerKilled {
  entity: Entity
}

These events come in two flavors:

    Event for global events defined with a GlobalTrigger
    EntityEvent for entity specific events defined with an EntityTrigger

The Trigger type is what Bevy uses to define the order in which observers run and what data they are passed. This is defined within the type system which helps to prevent misuse between different event flavors.

GlobalTrigger runs all global observers, the ones defined on the App that match the specific event it is defined on. In fact, the EntityTrigger does the same thing as GlobalTrigger, but additionally runs any entity scoped triggers.

An Observer is the callback system that receives these events. Observers also happens to come in two distinct categories:

    Broadcast observers
    Entity observers

To call these callbacks, we need to trigger events that they are interested in:

// Triggering a broadcast observer
commands.trigger(SomeEvent)

// Triggering an entity observer works the same way
commands.trigger(SomeEntityEvent { entity })

In addition to our own custom events, Bevy also has built-in events for each part of the component life-cycle:
Type	Description
Add	Triggers when a component is added
Insert	Triggers when a component is inserted
Replace	Triggers when a component is replaced
Remove	Triggers when a component is removed
Despawn	Triggers when a component is despawned
Broadcast observers

If we want a broadcast observer that listens to events globally we can add the observer to our App definition:

#[derive(Component, Debug)]
struct Position(Vec2);

#[derive(Component)]
struct Enemy;

fn on_respawn(
  event: On<Add, Enemy>,
  query: Query<(&Enemy, &Position)>,
) {
  let (enemy, position) = query.get(event.entity).unwrap();
  println!("Enemy was respawned at {:?}", position);
}

fn main() {
  App::new().add_plugins(DefaultPlugins).add_observer(on_respawn);
}

Any time a Enemy component is added, our on_respawn system will fire. Observer based systems must have On as their first argument.
Entity observers

If we use an entity observer we can choose to react to a events that are triggered on a specific entity.

We define EntityEvent and provide an entity field:

#[derive(EntityEvent)]
struct BossSpawned {
  entity: Entity,
}

Then we need to define an observer

fn on_boss_spawned(
  event: On<BossSpawned>,
  query: Query<(&Enemy, &Position)>,
) {
  if let Ok((enemy, position)) = query.get(event.entity) {
    println!("Boss was spawned at {:?}", position);
  }
}

When we spawn the entity, we can add this observer to that specific entity. Later, in any other system (or the same one like below) when you trigger your event with that entity, that observer will run.

fn spawn_boss(mut commands: Commands) {
  let entity = commands.spawn((Enemy, Boss)).observe(on_boss_spawned).id();

  commands.trigger(BossSpawned { entity });
}

These entity events will bubble up a hierarchy of ChildOf attached components.

When events are propagated they are re-sent to their next target while keeping track of where they started through original_target. This propagation continues until the chain reaches a dead-end, or the observer handling the propagation manually stops it.
Triggers

The generic arguments for On can be somewhat confusing:

pub struct On<'w, E, B: Bundle = ()> {
  // ...
}

The first generic argument E is the event.

The second generic argument B is optional. A good mental model is to think that the second generic argument is only for Bevy's built-in triggers. Any observers you create will only take one.

You should be aware that when using Bevy's built in trigger events the second generic B can be a bundle of components, and that it is going to act as a filter of those components using OR not AND logic.

So this observer:

fn on_respawn(
  event: On<Add, (Enemy, Person)>,
  // ...
)

Is going to trigger when an Enemy OR a Person is added.
Event propagation

You can set events to automatically propagate themselves according to a relationship.

#[derive(EntityEvent)]
#[entity_event(auto_propagate, propagate = &'static ChildOf)]
struct LocationTravelled {
  #[event_target]
  ship: Entity,
}

This would mean that a LocationTravelled event for a specific entity would be automatically triggered again for a related entity if we held a ChildOf(Entity) component.

#[derive(Component, Default)]
struct Ship;

#[derive(Component)]
struct Player;

fn spawn_player(mut commands: Commands) {
  // Adding `Player` as a child of `Ship`
  let ship = commands.spawn((Ship, children![Player])).id();

  // Trigger the event on the ship, it will propagate up to
  // the vehicle and player.
  commands.trigger(LocationTravelled { ship });
}

In this example, even though we triggered the event on the Ship, an additional event for the Player would also be sent to any relevant observers.

This can be useful in situations like picking where the actual event triggers on something like a Mesh but you want to handle the event on a related component somewhere in its hierarchy.
Choosing messages vs events

Events can be easier to reason about if we care about the effects of our events happening within a single frame.

Observers are processed when a key (the On trigger) is used to lookup a set of values (the observing systems) which are iterated in an arbitrary order and handled immediately.

Another good use case for observers is events we want to handle from only certain entities and not others of the same type as they can be triggered per entity.

On the other hand, systems that read Messages<T> can be put in a specific order.

Observers in 0.17 cannot be explicitly ordered. This becomes a problem if your observers depend on other observers having run before them.

Messages also help you decouple systems from each other. We can send a message and not have to control who exactly consumes our event. For observers, the producers of the event need to explicitly know who to trigger the event for.

This table shows the key differences:
	Events	Messages
Optimal event frequency	Infrequent	Frequent
Handler	Only handles a single event	Can handle many messages together
Latency	Immediate	Up to 1 frame
Event propagation	Bubbling	None
Scope	World or Entity	World
Ordering	No explicit order	Ordered
Coupling	High	Low

There can be multiple ways of defining the same behavior between the two so it is more up to your game's specific constraints.

Examples of good use cases for observers:

    Input handling (see bevy_enhanced_input) because of the timing

Examples of good use cases for events:

    Analytics because we don't care about the immediate effects
    Dependency injection through an event interface because we care about decoupling
