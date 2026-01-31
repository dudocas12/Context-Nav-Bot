from google import genai
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
        "desc": "Home base, my house, sleeping, delivery drop-off, safety." # approved
    },
    "park": {
        "coords": (0.0, 0.0), 
        "desc": "Nature, walking, running, trees, grass, relaxing, outside." # approved
    },
    "shopping_mall": {
        "coords": (1.3, 55.8), # approved
        "desc": "Shopping mall, shops, buying food, restaurants, shopping, supplies."
    },
    "hospital": {
        "coords": (-88, 4.98), 
        "desc": "Hospital, medical care, emergency, doctors, nurses, supplies."}, # approved
    "restaurant": {
        "coords": (87.2, -59.8), 
        "desc": "Restaurant, food, eating, lunch, dinner, supplies." # approved
    },
    "gas_station": {
        "coords": (84.6, -72.4), # approved
        "desc": "Gas station, fuel, car, supplies."
    },
    "office": {
        "coords": (-72.9, 68.9), # approved
        "desc": "Office, work, workplace"
    },
    "museum": {
        "coords": (82.3, 90.5), # approved
        "desc": "Museum, art, culture, history, exhibitions"
    },
    "church": {
        "coords": (-91.5, -80.6), # approved
        "desc": "Church, religion, worship, prayer, community"
    },
}     

def decide_destination(user_text):
    """ Returns a dictionary: {'place_name': (x, y)} """
    default_response = {"residential": (0.0, 0.0)}
    
    if not user_text:
        return default_response
    if not GEMINI_API_KEY:
        print("⚠️ No API Key found.")
        return default_response

    try:
        # ✅ NEW SDK SYNTAX
        client = genai.Client(api_key=GEMINI_API_KEY)
        
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
        
        # ✅ NEW GENERATION CALL
        response = client.models.generate_content(
            model='gemini-2.5-flash', 
            contents=prompt
        )
        
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
        return default_response