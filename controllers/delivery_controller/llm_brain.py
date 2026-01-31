import google.generativeai as genai
import json

# ==============================================================================
# 🔐 IMPORT SECRET KEY
# ==============================================================================
try:
    from my_secrets import GEMINI_API_KEY
except ImportError:
    GEMINI_API_KEY = None

ZONES = {
    "residential": {
        "coords": (85.2, -5.14), 
        "desc": "Home base, my house, sleeping, delivery drop-off, safety."
    },
    "park": {
        "coords": (0.0, 0.0), 
        "desc": "Nature, walking, running, trees, grass, relaxing, outside."
    },
    "commercial": {
        "coords": (-62, 6), 
        "desc": "Supermarket, grocery store, buying food, restaurants, shopping, supplies."
    }
}

def decide_destination(user_text):
    """ Returns a dictionary: {'place_name': (x, y)} """
    default_response = {"residential": (85.2, -5.14)}
    
    if not user_text:
        return default_response
    if not GEMINI_API_KEY:
        print("⚠️ No API Key found.")
        return default_response

    try:
        genai.configure(api_key=GEMINI_API_KEY)
        model = genai.GenerativeModel('gemini-2.5-flash-lite')
        
        locations_str = ""
        for name, data in ZONES.items():
            locations_str += f"- '{name}': located at {data['coords']}. Context: {data['desc']}\n"

        prompt = f"""
        You are the navigation brain for a robot.
        
        Known Map Coordinates:
        {locations_str}
        
        User Request: "{user_text}"
        
        Task: Identify the best location and return its exact coordinates.
        
        STRICT OUTPUT RULES:
        1. Return ONLY a valid JSON object.
        2. Format: {{"place_name": [x, y]}}
        3. Do NOT write markdown, explanations, or code blocks. Just the raw JSON string.
        4. If unsure, return home: {{"residential": [0.0, 0.0]}}
        """
        
        print(f"☁️ Sending '{user_text}' to Gemini...")
        response = model.generate_content(prompt)
        raw_text = response.text.strip()
        
        if raw_text.startswith("```"):
            raw_text = raw_text.strip("`").replace("json", "").strip()
            
        data = json.loads(raw_text)
        
        place_name = list(data.keys())[0]
        coords_list = data[place_name]
        
        final_coords = (float(coords_list[0]), float(coords_list[1]))
        
        print(f"🧠 AI DECISION: {place_name} at {final_coords}")
        return {place_name: final_coords}

    except Exception as e:
        print(f"❌ AI/JSON Error: {e}")
        print(f"   Raw output was: '{raw_text}'")
        return default_response