use async_tungstenite::tungstenite;
use futures::StreamExt;

fn main() {
    smol::block_on(run());
}

async fn run() {
    let url = "ws://localhost:8080";
    println!("Connecting to {}...", url);

    // Give the server a moment to start if running simultaneously
    smol::Timer::after(std::time::Duration::from_secs(1)).await;

    match async_tungstenite::async_std::connect_async(url).await {
        Ok((mut ws_stream, _)) => {
            println!("Connected! Listening for the heartbeat of the world...");

            while let Some(msg) = ws_stream.next().await {
                match msg {
                    Ok(tungstenite::Message::Text(text)) => {
                        // Parse JSON to verify structure if possible, otherwise just print
                        if let Ok(json) = serde_json::from_str::<serde_json::Value>(&text) {
                            if let Some(objects) = json.get("objects") {
                                if let Some(arr) = objects.as_array() {
                                    if !arr.is_empty() {
                                        println!(
                                            "\n[Timestamp {}]",
                                            json.get("timestamp")
                                                .unwrap_or(&serde_json::Value::Null)
                                        );
                                        for obj in arr {
                                            let id = obj.get("id").unwrap();
                                            let pos = obj.get("centroid").unwrap();
                                            println!("  > Object {}: {}", id, pos);
                                        }
                                    }
                                }
                            } else {
                                println!("Received: {}", text);
                            }
                        } else {
                            println!("Received raw: {}", text);
                        }
                    }
                    Ok(tungstenite::Message::Close(_)) => {
                        println!("Connection closed");
                        break;
                    }
                    Err(e) => {
                        println!("Error: {}", e);
                        break;
                    }
                    _ => {}
                }
            }
        }
        Err(e) => {
            println!("Failed to connect: {}", e);
        }
    }
}
